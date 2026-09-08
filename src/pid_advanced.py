class PIDAdvanced:
    """
    PID avancado com anti-windup, filtro derivativo e rate limiting.
    
    Funcionalidades adicionais em relacao ao PID basico:
    - Anti-windup: limita acumulador integral
    - Filtro derivativo: reduz ruido no termo D
    - Derivative on measurement: reduz spikes na saida
    - Output rate limit: limita velocidade de variacao da saida
    """
    
    def __init__(self, kp, ki, kd, setpoint=0):
        """
        Inicializa o PID avancado.
        
        Args:
            kp: Ganho proporcional
            ki: Ganho integral
            kd: Ganho derivativo
            setpoint: Valor desejado
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.integral = 0
        self.previous_error = 0
        self.previous_measurement = 0
        
        self.output_min = -255
        self.output_max = 255
        
        self.integral_min = -100
        self.integral_max = 100
        
        self.derivative_filter_alpha = 0.3
        self.filtered_derivative = 0
        
        self.rate_limit = 50
        self.previous_output = 0
        
        self.derivative_on_measurement = False
        
        self._first_run = True
        
    def compute(self, measurement):
        """
        Calcula saida PID com todas as melhorias.
        
        Args:
            measurement: Valor medido
            
        Returns:
            float: Saida de controle
        """
        error = self.setpoint - measurement
        
        self.integral += error
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        
        if self._first_run:
            raw_derivative = 0
            self._first_run = False
        elif self.derivative_on_measurement:
            raw_derivative = -(measurement - self.previous_measurement)
        else:
            raw_derivative = error - self.previous_error
            
        self.filtered_derivative = (
            self.derivative_filter_alpha * raw_derivative +
            (1 - self.derivative_filter_alpha) * self.filtered_derivative
        )
        
        self.previous_error = error
        self.previous_measurement = measurement
        
        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
        
        output = max(self.output_min, min(self.output_max, output))
        
        if self.rate_limit > 0:
            diff = output - self.previous_output
            if abs(diff) > self.rate_limit:
                diff = self.rate_limit if diff > 0 else -self.rate_limit
            output = self.previous_output + diff
            
        self.previous_output = output
        
        return output
    
    def reset(self):
        """Reseta todos os estados internos."""
        self.integral = 0
        self.previous_error = 0
        self.previous_measurement = 0
        self.filtered_derivative = 0
        self.previous_output = 0
        self._first_run = True
        
    def set_gains(self, kp, ki, kd):
        """Atualiza ganhos."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def set_limits(self, min_val, max_val):
        """Define limites de saida."""
        self.output_min = min_val
        self.output_max = max_val
        
    def set_integral_limits(self, min_val, max_val):
        """
        Define limites do acumulador integral (anti-windup).
        
        Args:
            min_val: Limite inferior do integrador
            max_val: Limite superior do integrador
        """
        self.integral_min = min_val
        self.integral_max = max_val
        
    def set_derivative_filter(self, alpha):
        """
        Define constante do filtro passa-baixa no derivativo.
        
        Args:
            alpha: Fator de suavizacao (0=sem filtro, 1=muito filtrado)
        """
        self.derivative_filter_alpha = max(0, min(1, alpha))
        
    def set_rate_limit(self, rate):
        """
        Define limite de variacao da saida.
        
        Args:
            rate: Variacao maxima por ciclo
        """
        self.rate_limit = rate
        
    def set_derivative_on_measurement(self, enabled):
        """
        Ativa/desativa calculo da derivativa sobre a medicao.
        
        Args:
            enabled: True para usar medicao, False para usar erro
        """
        self.derivative_on_measurement = enabled
        
    def get_components(self, measurement):
        """Retorna componentes P, I e D separadamente."""
        error = self.setpoint - measurement
        
        p = self.kp * error
        i = self.ki * self.integral
        
        if self._first_run:
            d = 0
        elif self.derivative_on_measurement:
            d = self.kd * (-(measurement - self.previous_measurement))
        else:
            d = self.kd * (error - self.previous_error)
            
        return {
            "p": p,
            "i": i,
            "d": d,
            "total": p + i + d,
            "error": error,
            "integral_accumulator": self.integral
        }
