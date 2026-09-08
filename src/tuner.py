from .pid import PID
import time


class PIDTuner:
    """
    Sintonizacao automatica de ganhos PID.
    
    Implementa o metodo de Ziegler-Nichols para encontrar
    ganhos otimos sem necessidade de sintonizacao manual.
    
    Procedimento:
    1. Define Ki e Kd como zero
    2. Aumenta Kp gradualmente ate oscilacao sustentada
    3. Registra Ku (ganho critico) e Tu (periodo de oscilacao)
    4. Calcula ganhos usando formulas de Ziegler-Nichols
    """
    
    ZIEGLER_NICHOLS = {
        "P":  lambda ku, tu: {"kp": 0.5 * ku, "ki": 0, "kd": 0},
        "PI": lambda ku, tu: {"kp": 0.45 * ku, "ki": 1.2 * ku / tu, "kd": 0},
        "PID": lambda ku, tu: {"kp": 0.6 * ku, "ki": 2 * ku / tu, "kd": ku * tu / 8},
    }
    
    def __init__(self, setpoint=0, output_min=-255, output_max=255):
        """
        Inicializa o sintonizador.
        
        Args:
            setpoint: Setpoint para teste
            output_min: Saida minima
            output_max: Saida maxima
        """
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        
        self.ku = None
        self.tu = None
        
    def find_critical_gain(self, plant_func, kp_start=0.1, kp_step=0.1, 
                           kp_max=50, test_duration=5, oscillation_threshold=3):
        """
        Encontra o ganho critico Ku.
        
        Args:
            plant_func: Funcao que simula a planta (recebe saida, retorna medicao)
            kp_start: Kp inicial
            kp_step: Incremento de Kp
            kp_max: Kp maximo para teste
            test_duration: Duracao do teste em segundos
            oscillation_threshold: Numero minimo de oscilacoes
            
        Returns:
            float: Ganho critico Ku
        """
        print("Procurando ganho critico (Ku)...")
        
        kp = kp_start
        
        while kp <= kp_max:
            pid = PID(kp=kp, ki=0, kd=0, setpoint=self.setpoint)
            pid.set_limits(self.output_min, self.output_max)
            
            oscillations = 0
            prev_sign = None
            start_time = time.time()
            
            measurement = self.setpoint
            
            while time.time() - start_time < test_duration:
                output = pid.compute(measurement)
                measurement = plant_func(output)
                
                current_sign = 1 if output > 0 else -1
                
                if prev_sign is not None and current_sign != prev_sign:
                    oscillations += 1
                    
                prev_sign = current_sign
                
                time.sleep(0.01)
                
            if oscillations >= oscillation_threshold:
                self.ku = kp
                print(f"Ku encontrado: {kp} (oscilacoes: {oscillations})")
                return kp
                
            kp += kp_step
            
        print("Nao foi possivel encontrar Ku no intervalo especificado.")
        return None
        
    def measure_oscillation_period(self, plant_func, ku, test_duration=10):
        """
        Mede o periodo de oscilacao com Ku.
        
        Args:
            plant_func: Funcao da planta
            ku: Ganho critico
            test_duration: Duracao do teste
            
        Returns:
            float: Periodo de oscilacao Tu
        """
        print(f"Medindo periodo de oscilacao com Ku={ku}...")
        
        pid = PID(kp=ku, ki=0, kd=0, setpoint=self.setpoint)
        pid.set_limits(self.output_min, self.output_max)
        
        crossings = []
        prev_error = None
        start_time = time.time()
        
        measurement = self.setpoint
        
        while time.time() - start_time < test_duration:
            output = pid.compute(measurement)
            measurement = plant_func(output)
            
            error = self.setpoint - measurement
            
            if prev_error is not None:
                if prev_error < 0 and error >= 0:
                    crossings.append(time.time())
                elif prev_error > 0 and error <= 0:
                    crossings.append(time.time())
                    
            prev_error = error
            time.sleep(0.01)
            
        if len(crossings) < 2:
            print("Nao foi possivel medir periodo.")
            return None
            
        periods = []
        for i in range(1, len(crossings)):
            period = crossings[i] - crossings[i-1]
            periods.append(period)
            
        self.tu = sum(periods) / len(periods)
        
        print(f"Tu medido: {self.tu:.4f}s")
        return self.tu
        
    def calculate_gains(self, method="PID"):
        """
        Calcula ganhos usando metodo de Ziegler-Nichols.
        
        Args:
            method: "P", "PI" ou "PID"
            
        Returns:
            dict: Ganhos calculados
        """
        if self.ku is None or self.tu is None:
            raise ValueError("Execute find_critical_gain e measure_oscillation_period primeiro.")
            
        formula = self.ZIEGLER_NICHOLS.get(method.upper())
        
        if formula is None:
            raise ValueError(f"Metodo invalido: {method}. Use P, PI ou PID.")
            
        gains = formula(self.ku, self.tu)
        
        print(f"\nGanhos calculados ({method}):")
        print(f"  Kp = {gains['kp']:.4f}")
        print(f"  Ki = {gains['ki']:.4f}")
        print(f"  Kd = {gains['kd']:.4f}")
        
        return gains
        
    def auto_tune(self, plant_func, method="PID", kp_start=0.1, kp_step=0.1):
        """
        Sintonizacao completa automatica.
        
        Args:
            plant_func: Funcao da planta
            method: Metodo de sintonizacao
            kp_start: Kp inicial para busca
            kp_step: Incremento de Kp
            
        Returns:
            PID: Controlador sintonizado
        """
        print("=" * 50)
        print("SINTONIZACAO AUTOMATICA - ZIEGLER-NICHOLS")
        print("=" * 50)
        print()
        
        ku = self.find_critical_gain(plant_func, kp_start, kp_step)
        
        if ku is None:
            print("Falha na sintonizacao.")
            return None
            
        tu = self.measure_oscillation_period(plant_func, ku)
        
        if tu is None:
            print("Falha na medicao de periodo.")
            return None
            
        gains = self.calculate_gains(method)
        
        pid = PID(
            kp=gains["kp"],
            ki=gains["ki"],
            kd=gains["kd"],
            setpoint=self.setpoint
        )
        pid.set_limits(self.output_min, self.output_max)
        
        print("\nControlador sintonizado criado com sucesso.")
        
        return pid


def simulate_plant(output, gain=1.0, delay=0.1, noise=0.0):
    """
    Simula planta simples para teste do sintonizador.
    
    Args:
        output: Saida do controlador
        gain: Ganho da planta
        delay: Atraso em segundos
        noise: Nivel de ruido
        
    Returns:
        float: Medicao simulada
    """
    import random
    
    time.sleep(delay)
    
    measurement = output * gain
    
    if noise > 0:
        measurement += random.uniform(-noise, noise)
        
    return measurement
