# Control Panel - TauraFly

Uma GCS, desenvolvida para o monitoramento e operação de drones da equipe TauraBots. O painel permite alternar perfeitamente entre a telemetria/vídeo de hardware real (ESP32-CAM, neste caso, mas será alterada provavelmente) e o ambiente de simulação (Gazebo/ArduPilot SITL).

## 🚀 Funcionalidades

- Modo Híbrido: Seleção em tempo real entre entrada de hardware (HTTP Stream) e simulação (ROS 2 Topics).

- Monitoramento MAVLink:

- Status de conexão do MAVROS em tempo real.

- Indicador visual de Pre-Arm Check (Verde para OK, Vermelho para falha).

- Exibição do Modo de Voo (GUIDED, LOITER, STABILIZE, etc.).

- Telemetria Local: Visualização precisa de coordenadas XYZ e Altitude Relativa.

- Logs do Sistema: Console estilo terminal para monitoramento de mensagens do ArduPilot e status da infraestrutura.

## 🛠️ Tecnologias Utilizadas

**Linguagem**: Python 3

**GUI**: PyQt6

**Robótica**: ROS 2 Humble & MAVROS

**Comunicação**: MAVLink Protocol

**Visão**: OpenCV (cv_bridge & Aruco)

**Hardware Suportado**: Pixhawk/Orange Cube & ESP32-CAM

## 📂 Estrutura do Projeto

```bash
.
├── taura-flight-command.py   # Script principal da interface
├── file.jpeg                 # Logo da TauraBots / Ícone do sistema
└── README.md                 # Documentação do projeto
```

## 🔧 Instalação e Requisitos

Certifique-se de estar em um ambiente com Ubuntu 22.04 e ROS 2 Humble instalado.

Instale as dependências de sistema:

```bash
sudo apt install ros-humble-mavros ros-humble-cv-bridge
pip install PyQt6 opencv-python numpy psutil
```

Configure o workspace do drone:

```bash
cd ~/colcon_ws
source install/setup.bash
```

Execute o painel:

```bash
python3 taura-flight-command.py
```

## 🎮 Como Usar
### Modo Simulação (Gazebo)

- Inicie sua simulação (ex: ros2 launch taura-drone-package start_sim.launch.py).

- No painel, selecione SIMULAÇÃO.

- O status do MAVROS deve ficar verde e a telemetria começará a oscilar conforme o drone se move.

### Modo Hardware (Real)

- Estar na mesma rede da ESPCAM, verificar no exemplo CameraWebserver do ArduinoIDE O IP que serve.

- Selecione HARDWARE e clique em CONECTAR VÍDEO.

