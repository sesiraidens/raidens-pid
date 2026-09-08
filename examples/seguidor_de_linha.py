"""
Exemplo de PID para seguidor de linha.

Simula um robo que segue uma linha preta usando sensores IR.
"""
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.pid import PID


class LineFollower:
    """
    Simulador de seguidor de linha com PID.
    
    Simula um robo com 5 sensores IR que segue uma linha preta.
    """
    
    def __init__(self):
        self.pid = PID(kp=2.5, ki=0.8, kd=0.3, setpoint=0)
        self.pid.set_limits(-255, 255)
        
        self.position = 0
        self.speed = 100
        
    def read_sensors(self):
        """
        Simula leitura dos sensores.
        
        Valores:
            -2 = linha muito a esquerda
            -1 = linha a esquerda
             0 = linha no centro
             1 = linha a direita
             2 = linha muito a direita
        """
        return self.position
        
    def set_motors(self, left_speed, right_speed):
        """Simula comando dos motores."""
        print(f"  Motores: E={left_speed:+4d} | D={right_speed:+4d} | Pos={self.position:+d}")
        
    def run(self, duration=10):
        """
        Executa o seguidor de linha.
        
        Args:
            duration: Duracao em segundos
        """
        print("=" * 50)
        print("SEGUIDOR DE LINHA COM PID")
        print("=" * 50)
        print()
        print("Simulacao do robo seguindo linha preta.")
        print("Posicao: -2=extrema esquerda, 0=centro, 2=extrema direita")
        print()
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            sensor_value = self.read_sensors()
            
            correction = self.pid.compute(sensor_value)
            
            left_speed = self.speed + correction
            right_speed = self.speed - correction
            
            left_speed = max(-255, min(255, left_speed))
            right_speed = max(-255, min(255, right_speed))
            
            self.set_motors(left_speed, right_speed)
            
            self.position = self._simulate_movement(left_speed, right_speed)
            
            time.sleep(0.1)
            
        print()
        print("Simulacao finalizada.")
        
    def _simulate_movement(self, left_speed, right_speed):
        """Simula movimento do robo."""
        diff = left_speed - right_speed
        
        new_position = self.position + (diff / 500.0)
        
        return max(-2, min(2, round(new_position)))


def main():
    follower = LineFollower()
    follower.run(duration=10)


if __name__ == "__main__":
    main()
