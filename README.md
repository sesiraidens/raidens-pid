<div align="center">

<img src="https://sesiraidens.github.io/portifolio/assets/logo_color-aNRVU26Y.png" width="80">

# raidens-pid

Implementacoes de controlador PID para navegacao autonoma de robos.

![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-d9333b?style=flat)
![Status](https://img.shields.io/badge/Status-Active-2ea043?style=flat)

</div>

---

## Sobre

O **raidens-pid** implementa controladores PID (Proporcional, Integral, Derivativo) para tarefas comuns de robotica: seguir linha, manter distancia e manter angulo.

### Como funciona o PID

O PID calcula uma saida de controle baseada em tres termos:

- **P (Proporcional):** Reage ao erro atual. Se o erro e grande, a correcao e grande.
- **I (Integral):** Acumula erros passados. Elimina erro estacionario que o P nao resolve.
- **D (Derivativo):** Preve tendencias do erro. Suaviza a resposta e reduz overshoot.

**Formula:** saida = Kp * erro + Ki * integral(erro) + Kd * derivada(erro)

---

## Estrutura

`
raidens-pid/
├── src/
│   ├── pid.py              # PID basico
│   ├── pid_advanced.py     # PID com anti-windup e filtros
│   ├── cascade_pid.py      # PID em cascade (duas camadas)
│   ├── tuner.py            # Sintonizacao automatica
│   └── __init__.py
├── examples/
│   ├── seguidor_de_linha.py
│   ├── controlador_de_distancia.py
│   └── controlador_de_angulo.py
├── configs/
│   ├── pid_linha.yaml
│   └── pid_parede.yaml
└── README.md
`

---

## Modulos

### pid.py - PID Basico

`python
from src.pid import PID

pid = PID(kp=2.5, ki=0.8, kd=0.3, setpoint=0)
saida = pid.compute(medicao)
`

**Metodos:**
- compute(measurement) - Calcula saida PID
- eset() - Zera acumuladores
- set_gains(kp, ki, kd) - Atualiza ganhos
- set_limits(min, max) - Define limites de saida
- get_components(measurement) - Retorna P, I, D separados

### pid_advanced.py - PID Avancado

Adiciona funcionalidades ao PID basico:

- **Anti-windup:** Limita acumulador integral para evitar saturacao
- **Filtro derivativo:** Suaviza o termo D para reduzir ruido
- **Derivative on measurement:** Calcula D sobre medicao ao inves de erro
- **Rate limit:** Limita velocidade de variacao da saida

`python
from src.pid_advanced import PIDAdvanced

pid = PIDAdvanced(kp=2.0, ki=0.5, kd=0.2)
pid.set_integral_limits(-50, 50)
pid.set_derivative_filter(0.3)
pid.set_rate_limit(30)
`

### cascade_pid.py - PID em Cascade

Duas camadas de controle para sistemas de dois estagios.

`python
from src.cascade_pid import create_distance_controller

controller = create_distance_controller()
saida = controller.compute(distancia_parede, velocidade_motor)
`

**Exemplos:**
- create_distance_controller() - Controle de distancia de parede
- create_line_controller() - Seguidor de linha

### tuner.py - Sintonizacao Automatica

Metodo de Ziegler-Nichols para encontrar ganhos otimos.

`python
from src.tuner import PIDTuner

tuner = PIDTuner(setpoint=0)
pid = tuner.auto_tune(plant_func, method="PID")
`

**Procedimento:**
1. Aumenta Kp ate oscilacao sustentada (Ku)
2. Mede periodo de oscilacao (Tu)
3. Calcula ganhos: Kp=0.6*Ku, Ki=2*Ku/Tu, Kd=Ku*Tu/8

---

## Tabelas de Ganhos

### Ziegler-Nichols

| Metodo | Kp | Ki | Kd |
|---|---|---|---|
| P | 0.5 * Ku | - | - |
| PI | 0.45 * Ku | 1.2 * Kp / Tu | - |
| PID | 0.6 * Ku | 2 * Kp / Tu | Kp * Tu / 8 |

### Ganhos Recomendados

| Aplicacao | Kp | Ki | Kd |
|---|---|---|---|
| Seguidor de linha | 2.5 | 0.8 | 0.3 |
| Controle de distancia | 2.0 | 0.5 | 0.2 |
| Controle de angulo | 3.0 | 0.5 | 1.0 |

---

## Exemplos

### Seguidor de Linha

`ash
python examples/seguidor_de_linha.py
`

Simula robo com 5 sensores IR seguindo linha preta. PID ajusta direcao dos motores.

### Controle de Distancia

`ash
python examples/controlador_de_distancia.py
`

Simula robo mantendo 30cm de distancia de parede. Usa PID avancado com anti-windup.

### Controle de Angulo

`ash
python examples/controlador_de_angulo.py
`

Simula robo mantendo orientacao de 0 graus. Usa controle diferencial de motores.

---

## Dependencias

Nenhuma dependencia externa. Apenas Python 3.7+ padrao.

---

## Equipe

**RAIDENS - SESI Aluminio 192**

Desenvolvido para uso interno da equipe. Licenciado sob MIT.