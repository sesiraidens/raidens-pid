"""
Exemplo de PID para controle de distancia.

Simula um robo que mantem distancia constante de uma parede.
"""
import sys
import os
import time
import random

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.pid import PID
from src.pid_advanced import PIDAdvanced


class DistanceController:
    """
    Simulador de controle de distancia com PID.
    
    Simula um robo que mantem 30cm de distancia de uma parede.
    """
    
    def __init__(self, target_distance=30):
        self.pid = PIDAdvanced(kp=2.0, ki=0.5, kd=0.2, setpoint=target_distance)
        self.pid.set_limits(-255, 255)
        self.pid.set_integral_limits(-50, 50)
        self.pid.set_derivative_filter(0.3)
        self.pid.set_rate_limit(30)
        
        self.current_distance = target_distance
        self.speed = 0
        
    def read_ultrasonic(self):
        """Simula leitura do sensor ultrassonico."""
        return self.current_distance + random.uniform(-0.5, 0.5)
        
    def set_motors(self, speed):
        """Simula comando dos motores."""
        self.speed = speed
        print(f"  Distancia: {self.current_distance:.1f}cm | Velocidade: {speed:+4d}")
        
    def update_position(self, dt=0.1):
        """Atualiza posicao do robo."""
        movement = self.speed * 0.01 * dt
        self.current_distance -= movement
        
        self.current_distance += random.uniform(-0.2, 0.2)
        
    def run(self, duration=10, target=30):
        """
        Executa o controle de distancia.
        
        Args:
            duration: Duracao em segundos
            target: Distancia alvo em cm
        """
        print("=" * 50)
        print("CONTROLE DE DISTANCIA COM PID")
        print("=" * 50)
        print()
        print(f"Distancia alvo: {target}cm")
        print()
        
        self.pid.set_setpoint(target)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            distance = self.read_ultrasonic()
            
            correction = self.pid.compute(distance)
            
            self.set_motors(int(correction))
            
            self.update_position()
            
            time.sleep(0.1)
            
        print()
        print("Simulacao finalizada.")
        
    def show_components(self, distance):
        """Mostra componentes PID."""
        components = self.pid.get_components(distance)
        print(f"  P={components['p']:.2f} | I={components['i']:.2f} | D={components['d']:.2f}")


def main():
    controller = DistanceController(target_distance=30)
    controller.run(duration=10, target=30)


if __name__ == "__main__":
    main()
