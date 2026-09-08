from .pid import PID


class CascadePID:
    """
    PID em cascade (duas camadas).
    
    Util para controle de dois estagios:
    - PID externo: controla a variavel principal
    - PID interno: controla a variavel secundaria
    
    Exemplo:
        - Externo: distancia de parede (setpoint: 30cm)
        - Interno: velocidade do motor
    """
    
    def __init__(self, outer_pid, inner_pid):
        """
        Inicializa o PID em cascade.
        
        Args:
            outer_pid: Instancia do PID externo
            inner_pid: Instancia do PID interno
        """
        self.outer = outer_pid
        self.inner = inner_pid
        
    def compute(self, outer_measurement, inner_measurement):
        """
        Calcula saida em cascade.
        
        Args:
            outer_measurement: Medicao da variavel principal
            inner_measurement: Medicao da variavel secundaria
            
        Returns:
            float: Saida de controle final
        """
        outer_output = self.outer.compute(outer_measurement)
        
        self.inner.set_setpoint(outer_output)
        
        inner_output = self.inner.compute(inner_measurement)
        
        return inner_output
    
    def reset(self):
        """Reseta ambos os PIDs."""
        self.outer.reset()
        self.inner.reset()
        
    def set_gains(self, outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd):
        """Atualiza ganhos de ambos os PIDs."""
        self.outer.set_gains(outer_kp, outer_ki, outer_kd)
        self.inner.set_gains(inner_kp, inner_ki, inner_kd)
        
    def set_outer_setpoint(self, setpoint):
        """Define setpoint do PID externo."""
        self.outer.set_setpoint(setpoint)
        
    def get_components(self, outer_measurement, inner_measurement):
        """Retorna componentes de ambos os PIDs."""
        return {
            "outer": self.outer.get_components(outer_measurement),
            "inner": self.inner.get_components(inner_measurement)
        }


def create_distance_controller(kp=2.0, ki=0.5, kd=0.2):
    """
    Cria controlador em cascade para manter distancia de parede.
    
    Args:
        kp: Ganho proporcional do externo
        ki: Ganho integral do externo
        kd: Ganho derivativo do externo
        
    Returns:
        CascadePID: Controlador configurado
    """
    outer = PID(kp=kp, ki=ki, kd=kd, setpoint=30)
    inner = PID(kp=1.5, ki=0.3, kd=0.1, setpoint=0)
    
    outer.set_limits(-100, 100)
    inner.set_limits(-255, 255)
    
    return CascadePID(outer, inner)


def create_line_controller(kp=2.5, ki=0.8, kd=0.3):
    """
    Cria controlador em cascade para seguir linha.
    
    Args:
        kp: Ganho proporcional
        ki: Ganho integral
        kd: Ganho derivativo
        
    Returns:
        CascadePID: Controlador configurado
    """
    outer = PID(kp=kp, ki=ki, kd=kd, setpoint=0)
    inner = PID(kp=1.0, ki=0.2, kd=0.05, setpoint=0)
    
    outer.set_limits(-100, 100)
    inner.set_limits(-255, 255)
    
    return CascadePID(outer, inner)
