"""
Exemplo de PID para controle de angulo.

Simula um robo que mantem orientacao fixa usando giroscopio.
"""
import sys
import os
import time
import random
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.pid import PID


class AngleController:
    """
    Simulador de controle de angulo com PID.
    
    Simula um robo que mantem angulo de 0 graus usando MPU6050.
    """
    
    def __init__(self, target_angle=0):
        self.pid = PID(kp=3.0, ki=0.5, kd=1.0, setpoint=target_angle)
        self.pid.set_limits(-255, 255)
        
        self.current_angle = 0
        self.angular_velocity = 0
        
    def read_gyroscope(self):
        """Simula leitura do giroscopio."""
        return self.current_angle + random.uniform(-0.5, 0.5)
        
    def set_motors(self, left_speed, right_speed):
        """Simula comando diferencial dos motores."""
        print(f"  Angulo: {self.current_angle:+.1f}graus | E={left_speed:+4d} D={right_speed:+4d}")
        
    def update_angle(self, left_speed, right_speed, dt=0.1):
        """Atualiza angulo do robo."""
        diff = left_speed - right_speed
        
        self.angular_velocity = diff * 0.1
        
        self.current_angle += self.angular_velocity * dt
        
        self.current_angle = self.current_angle % 360
        
        if self.current_angle > 180:
            self.current_angle -= 360
        elif self.current_angle < -180:
            self.current_angle += 360
            
    def run(self, duration=10, target=0):
        """
        Executa o controle de angulo.
        
        Args:
            duration: Duracao em segundos
            target: Angulo alvo em graus
        """
        print("=" * 50)
        print("CONTROLE DE ANGULO COM PID")
        print("=" * 50)
        print()
        print(f"Angulo alvo: {target} graus")
        print()
        
        self.pid.set_setpoint(target)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            angle = self.read_gyroscope()
            
            correction = self.pid.compute(angle)
            
            left_speed = int(-correction)
            right_speed = int(correction)
            
            self.set_motors(left_speed, right_speed)
            
            self.update_angle(left_speed, right_speed)
            
            time.sleep(0.1)
            
        print()
        print("Simulacao finalizada.")
        
    def wrap_angle(self, angle):
        """Normaliza angulo para [-180, 180]."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


def main():
    controller = AngleController(target_angle=0)
    controller.run(duration=10, target=0)


if __name__ == "__main__":
    main()
