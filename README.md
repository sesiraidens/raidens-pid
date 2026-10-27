<div align="center">

<img src="https://sesiraidens.github.io/portifolio/assets/logo_color-aNRVU26Y.png" alt="Logo RAIDENS" width="140">

# raidens-pid

**Controladores PID em Python para navegação autônoma de robôs**

![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=flat-square&logo=python&logoColor=white)
![Sem dependências](https://img.shields.io/badge/depend%C3%AAncias-nenhuma-2EA043?style=flat-square)
![Licença MIT](https://img.shields.io/badge/Licen%C3%A7a-MIT-D9333B?style=flat-square)

[Sobre](#sobre) · [Instalação](#instalação) · [Uso](#uso) · [Sintonia](#sintonia-dos-ganhos) · [Exemplos](#exemplos)

</div>

---

## Sobre

O **raidens-pid** reúne os controladores que a equipe RAIDENS usa para seguir linha, manter distância de parede e segurar o ângulo do robô. A ideia é ter um único lugar com implementações testadas, comentadas e com ganhos de partida, para que ninguém precise reescrever um PID do zero antes de cada competição.

O PID calcula a correção a partir de três termos:

| Termo | O que faz | Na prática |
|---|---|---|
| **P** · Proporcional | Reage ao erro atual | Erro grande, correção grande |
| **I** · Integral | Acumula o erro passado | Elimina o erro que sobra quando o robô "quase" chega |
| **D** · Derivativo | Olha a tendência do erro | Amortece a resposta e reduz o overshoot |

```text
saída = Kp · erro + Ki · ∫erro dt + Kd · d(erro)/dt
```

## Destaques

- **PID avançado** com anti-windup, filtro no termo derivativo, derivada sobre a medição e limite de taxa de variação
- **PID em cascata** para sistemas de dois estágios, com fábricas prontas para distância e linha
- **Sintonia automática** pelo método de Ziegler-Nichols
- **Configurações em YAML** com ganhos de partida para cada tarefa
- **Zero dependências**: só a biblioteca padrão do Python

## Instalação

```bash
git clone https://github.com/sesiraidens/raidens-pid.git
cd raidens-pid
```

Não há pacotes para instalar. Rode os scripts a partir da raiz do repositório para que o pacote `src` seja encontrado.

## Uso

### PID avançado

```python
from src.pid_advanced import PIDAdvanced

pid = PIDAdvanced(kp=2.0, ki=0.5, kd=0.2, setpoint=0)
pid.set_limits(-255, 255)           # saturação da saída
pid.set_integral_limits(-50, 50)    # anti-windup
pid.set_derivative_filter(0.3)      # suaviza o termo D
pid.set_rate_limit(30)              # variação máxima por ciclo

saida = pid.compute(medicao)
p, i, d = pid.get_components(medicao)
```

### PID em cascata

A malha externa decide *o que* o robô quer (por exemplo, a velocidade para chegar à distância certa) e a malha interna decide *como* chegar lá (o PWM do motor).

```python
from src.cascade_pid import create_distance_controller

controller = create_distance_controller(kp=2.0, ki=0.5, kd=0.2)
saida = controller.compute(distancia_parede, velocidade_motor)
```

Também existe `create_line_controller()` para seguidor de linha.

### Sintonia automática

```python
from src.tuner import PIDTuner, simulate_plant

tuner = PIDTuner(setpoint=0)
pid = tuner.auto_tune(simulate_plant, method="PID")
```

## Sintonia dos ganhos

O `PIDTuner` aumenta o Kp até a resposta oscilar de forma sustentada (ganho crítico **Ku**), mede o período dessa oscilação (**Tu**) e aplica a tabela de Ziegler-Nichols:

| Método | Kp | Ki | Kd |
|---|---|---|---|
| P | 0,5 · Ku | | |
| PI | 0,45 · Ku | 1,2 · Kp / Tu | |
| PID | 0,6 · Ku | 2 · Kp / Tu | Kp · Tu / 8 |

### Ganhos de partida

Valores que funcionaram bem nos nossos robôs. Use como ponto de partida e ajuste na pista.

| Aplicação | Kp | Ki | Kd | Arquivo |
|---|---|---|---|---|
| Seguidor de linha | 2,5 | 0,8 | 0,3 | `configs/pid_linha.yaml` |
| Distância de parede | 2,0 | 0,5 | 0,2 | `configs/pid_parede.yaml` |
| Controle de ângulo | 3,0 | 0,5 | 1,0 | |

## Exemplos

Todos os exemplos simulam o robô no terminal, então dá para testar a lógica sem hardware.

| Comando | O que demonstra |
|---|---|
| `python examples/seguidor_de_linha.py` | Robô com 5 sensores IR seguindo uma linha preta |
| `python examples/controlador_de_distancia.py` | Manter 30 cm da parede com PID avançado e anti-windup |
| `python examples/controlador_de_angulo.py` | Segurar a orientação em 0° com controle diferencial dos motores |

## Estrutura

```text
raidens-pid/
├── src/          # Controladores: básico, avançado, cascata e sintonizador
├── examples/     # Simulações prontas para rodar
└── configs/      # Ganhos de partida em YAML
```

## Ecossistema RAIDENS

Este repositório faz parte de um conjunto de bibliotecas da equipe. Elas usam os mesmos nomes de classes e convenções, então o que você aprende em uma serve nas outras.

| Repositório | Para quê |
|---|---|
| [`raidens-python`](https://github.com/sesiraidens/raidens-python) | Biblioteca principal em Python: sensores, motores e robôs prontos (seguidor, parede, resgate) |
| [`raidens-arduino`](https://github.com/sesiraidens/raidens-arduino) | As mesmas abstrações em C++ para rodar direto no Arduino |
| [`raidens-sensores`](https://github.com/sesiraidens/raidens-sensores) | Catálogo completo de sensores com calibração e filtragem |
| [`raidens-motores`](https://github.com/sesiraidens/raidens-motores) | Motores DC, servo, passo, driver L298N e rodas mecanum |
| [`raidens-opencv`](https://github.com/sesiraidens/raidens-opencv) | Visão computacional: cores, linhas e navegação por câmera |
| [`raidens-lab`](https://github.com/sesiraidens/raidens-lab) | Calibração interativa de cores em HSV e LAB |
| [`raidens-obr`](https://github.com/sesiraidens/raidens-obr) | Robô completo para a OBR, integrando sensores, motores e missões |
| [`raidens-electronics`](https://github.com/sesiraidens/raidens-electronics) | Cálculos e modelos de componentes eletrônicos |
| [`raidens-3d`](https://github.com/sesiraidens/raidens-3d) | Peças 3D paramétricas geradas em OpenSCAD |
| [`raidens-ros`](https://github.com/sesiraidens/raidens-ros) | Estrutura de nodes e tópicos ROS 2 |
| [`raidens-snippets`](https://github.com/sesiraidens/raidens-snippets) | Trechos de código prontos para copiar e colar |

---

<div align="center">

<img src="https://sesiraidens.github.io/portifolio/assets/logo_color-aNRVU26Y.png" alt="Logo RAIDENS" width="56">

**Equipe RAIDENS** · SESI 192 Alumínio/SP

[![Email](https://img.shields.io/badge/Email-sesiraidens%40gmail.com-D14836?style=flat-square&logo=gmail&logoColor=white)](mailto:sesiraidens@gmail.com)
[![Linktree](https://img.shields.io/badge/Linktree-sesiraidens-39E06B?style=flat-square&logo=linktree&logoColor=white)](https://linktr.ee/sesiraidens)
[![GitHub](https://img.shields.io/badge/GitHub-sesiraidens-181717?style=flat-square&logo=github&logoColor=white)](https://github.com/sesiraidens)

<sub>Desenvolvido pela equipe RAIDENS para fins educacionais. Licenciado sob MIT.</sub>

</div>

