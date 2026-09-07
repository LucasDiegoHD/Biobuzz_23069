# Visão Geral do Código do Robô

Este documento fornece uma visão geral detalhada da arquitetura de software do robô, construída
sobre o framework de comando da FTCLib. A estrutura é dividida em subsistemas, comandos e um
contêiner central que os une.

## Estrutura do Projeto

O código está organizado principalmente nas seguintes pastas:

- **`subsystems`**: Contém classes que representam os diferentes mecanismos de hardware do robô.
- **`commands`**: Contém classes que definem as ações e comportamentos do robô.
- **`robot`**: Contém a classe `RobotContainer`, que inicializa e conecta todos os componentes.
- **`autos`**: Contém a lógica e as trajetórias para os modos autônomos.

---

## Subsistemas

Os subsistemas são classes que encapsulam e gerenciam o hardware de um mecanismo específico do robô.
Eles expõem métodos para controlar o hardware sem expor os detalhes da implementação.

### DrivetrainSubsystem

- **Responsabilidade**: Gerencia o movimento do robô, incluindo a odometria (estimativa de posição)
  e o seguimento de trajetória.
- **Biblioteca Principal**: Utiliza a biblioteca `Pedro Pathing` para controle de movimento
  avançado.
- **Funcionalidades**: Fornece métodos para dirigir em modo teleoperado (field-centric), seguir
  caminhos autônomos e atualizar a visualização do robô no dashboard.

### IntakeSubsystem

- **Responsabilidade**: Controla o sistema de admissão de peças.
- **Componentes**: Gerencia um motor para o rolo de admissão e um motor de "gatilho" para alimentar
  as peças.
- **Métodos**: `run()`, `reverse()`, `stop()`, `runTrigger()`, `stopTrigger()`.

### ShooterSubsystem

- **Responsabilidade**: Controla o mecanismo de lançamento de peças.
- **Componentes**: Gerencia dois motores de lançamento (com controle de velocidade PIDF) e um servo
  para ajustar o ângulo do capô (hood).
- **Funcionalidades**: Define a velocidade de lançamento alvo, para os motores e ajusta a posição do
  capô.

### IndexerSubsystem

- **Responsabilidade**: Gerencia as peças dentro do robô, detectando sua presença e contagem.
- **Sensores**: Utiliza um sensor de entrada (beam break) e um sensor de saída (cor/distância).
- **Funcionalidades**: Rastreia o número de peças, verifica se está cheio e detecta a entrada/saída
  de peças.

### VisionSubsystem

- **Responsabilidade**: Processa a visão da câmera para detecção de AprilTags e estimativa de pose.
- **Hardware**: Utiliza uma câmera Limelight.
- **Funcionalidades**: Detecta alvos, calcula a distância e o ângulo até eles, e fornece uma
  estimativa da pose (posição e orientação) do robô no campo.

---

## Comandos

Os comandos implementam a lógica de controle do robô. Eles utilizam os métodos dos subsistemas para
realizar ações complexas.

### Comandos de Movimentação

- **`TeleOpDriveCommand`**: Comando padrão do drivetrain. Permite o controle `field-centric` durante
  o período teleoperado. O movimento do robô é relativo ao campo, não à sua própria orientação.
- **`TeleOpDriveAimingCommand`**: Variante do `TeleOpDriveCommand` que mira automaticamente em um
  ponto fixo no campo enquanto ainda permite que o piloto se mova.
- **`AlignToAprilTagCommand`**: Alinha o robô a um AprilTag visível usando um controle PID. Permite
  ao piloto manter o controle de movimento para frente/trás e para os lados.
- **`FollowPathCommand`**: Executa uma trajetória pré-definida (`PathChain`) da biblioteca
  `Pedro Pathing`.
- **`GoToPoseCommand`**: Move o robô para uma pose específica no campo criando uma trajetória
  simples em tempo real.

### Comandos de Ação

- **`ShootCommand`**: Orquestra a sequência completa de lançamento de uma peça, gerenciando uma
  máquina de estados para o transporte, aceleração e disparo.
- **`AutoShootCommand`**: Um grupo de comandos sequenciais que automatiza todo o processo de mira e
  tiro: alinha ao alvo (`AlignToAprilTagCommand`), espera o shooter atingir a velocidade (
  `WaitUntilCommand`) e aciona o gatilho.
- **`SpinShooterCommand`**: Define a velocidade dos motores de lançamento para uma de duas
  predefinições (tiro longo ou curto) ou os para.
- **`SetHoodPositionCommand`**: Ajusta o ângulo do capô do lançador para uma posição específica.

### Comandos de Utilidade

- **`UpdateLimelightYawCommand`**: Comando padrão do `VisionSubsystem`. Atualiza continuamente a
  orientação (yaw) do robô na Limelight usando dados da odometria do drivetrain, melhorando a
  precisão da estimativa de pose da câmera.

---

## RobotContainer

A classe `RobotContainer` é o ponto central do programa. Suas responsabilidades são:

1. **Inicializar Subsistemas**: Cria uma instância de cada subsistema.
2. **Configurar Controles**: Mapeia os botões dos gamepads (piloto e operador) para os comandos
   correspondentes. Por exemplo, o botão 'Y' do piloto ativa o `AlignToAprilTagCommand`, enquanto
   o 'B' do operador ativa o `SpinShooterCommand`.
3. **Definir Comandos Padrão**: Associa comandos que devem estar sempre em execução a determinados
   subsistemas (ex: `TeleOpDriveCommand` para o `DrivetrainSubsystem`).
4. **Fornecer Comandos Autônomos**: Disponibiliza os comandos completos para os diferentes modos
   autônomos (ex: `getAutonomousBlueRearCommand()`).

## Fluxo de Controle

1. **Inicialização**: Quando um OpMode é iniciado, ele cria uma instância do `RobotContainer`.
2. **Construção**: O construtor do `RobotContainer` inicializa todos os subsistemas e configura os
   mapeamentos de botões.
3. **Execução (TeleOp)**: O `CommandScheduler` da FTCLib executa continuamente o comando padrão do
   drivetrain (`TeleOpDriveCommand`). Quando um piloto pressiona um botão, o comando correspondente
   é agendado (por exemplo, `AlignToAprilTagCommand`), que pode interromper temporariamente o
   comando padrão.
4. **Execução (Autônomo)**: O OpMode autônomo solicita o comando autônomo apropriado do
   `RobotContainer` (ex: `getAutonomousBlueRearCommand()`) e o agenda para execução.
