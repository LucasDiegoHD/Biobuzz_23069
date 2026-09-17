# Diretrizes e Arquitetura de Tração: TechMaker (#23069)

Esta regra documenta a arquitetura de tração Mecanum e o sistema de pilotagem da equipe **TechMaker (#23069)**, unindo a física de movimentação da **RevAmped (#12808)** com as assistências inteligentes da **Cuberobot (#23641)**.

---

## 1. Identidade e Equipes de Referência Oficiais (Benchmarking)
- **Equipe**: TechMaker (#23069).
- **Equipes de Referência Oficiais**:
  1. **RevAmped Robotics (#12808)** (Repo: `RevAmped-Decode-V2`):
     - Referência para: Física de movimentação, controle direto nos `DcMotorEx` (bypass do calculador do Pedro), curva de sensibilidade tangente ($0.5 \cdot \tan(\text{stick} \cdot 1.12)$), snap de $15^\circ$ no dedão (`smoothGamepadAngle`), normalização por denominador (`denominator = Math.max(|y| + |x| + |rx|, 1.0)`) e cache de potência zero (`prevVelZero`).
  2. **Cuberobot (#23641)** (Repo: `FTC_Decode`):
     - Referência para: Automações inteligentes de pilotagem no TeleOp, modo **Field-Centric como padrão**, controle de orientação em malha fechada via PIDF (`headingPIDFController`), esquadrinhamento de $90^\circ$ nos gatilhos (`snapToNearestCardinal`), trava de alinhamento frontal de ciclo e arranque autônomo para a zona de arremesso (**Função Kick**).
- **Modo de Condução Padrão**: **Field-Centric** (`fieldCentric = true`).
- **Hardware Drivetrain**: Mecanum drive com 4 motores GoBilda Yellowjacket controlados via odometria GoBilda Pinpoint / Pedro Pathing v2.1.2 + Ivy v1.0.0.

---

## 2. Cinemática e Controle de Motores (Padrão RevAmped #12808)
- **Acesso Direto aos Motores**: O TeleOp **não** utiliza o `follower.setTeleOpDrive(...)` do Pedro Pathing (que causava latência e travava o giro ao transladar). O subsistema acessa os motores diretamente:
  ```java
  motors = ((Mecanum) follower.drivetrain).getMotors();
  leftFront = motors.get(0); leftRear = motors.get(1);
  rightFront = motors.get(2); rightRear = motors.get(3);
  ```
- **Direções dos Motores**:
  - `leftFront` e `leftRear`: `REVERSE`.
  - `rightFront` e `rightRear`: `FORWARD`.
  - `ZeroPowerBehavior`: `BRAKE`.
- **Sensibilidade Tangente**:
  $$y = -0.5 \cdot \tan(\text{stickY} \cdot 1.12), \quad x = 0.5 \cdot \tan(\text{stickX} \cdot 1.12), \quad rx = 0.5 \cdot \tan(\text{stickRX} \cdot 1.12)$$
  Garante resolução milimétrica no centro e potência máxima nas bordas.
- **Filtro de Desvio do Dedão (Snap 15°)**:
  `smoothGamepadAngle(x, y, angleZero)` trava eixos retos quando o desvio for menor que $\tan(15^\circ)$.
- **Normalização por Denominador (Giro e Translação Simultâneos)**:
  ```java
  double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
  double frontLeftPow = (y + x + rx) / denominator;
  double backLeftPow = (y - x + rx) / denominator;
  double frontRightPow = (y - x - rx) / denominator;
  double backRightPow = (y + x - rx) / denominator;
  ```
  Evita a saturação dos motores e garante que o robô translade a 100% de velocidade sem perder torque de giro.
- **Zero-Power Caching**: Variável `prevVelZero` evita comandos I2C/bulk desnecessários para a Control Hub quando o robô está em repouso.

---

## 3. Field-Centric e Calibração de Heading
- **Matriz de Rotação Perfeita**:
  $$\text{angle} = -(\text{robotHeading} - \text{fieldHeadingOffset})$$
  $$\text{rotX} = x \cos(\text{angle}) - y \sin(\text{angle})$$
  $$\text{rotY} = x \sin(\text{angle}) + y \cos(\text{angle})$$
- **START Button (Zerar Campo Sem Corromper Odometria)**:
  O botão START define `fieldHeadingOffset = follower.getHeading()`. Isso recalibra o "norte" do piloto para onde o robô está apontando no momento, **sem alterar nem corromper as coordenadas $(X, Y)$ da odometria Pinpoint**.
- **BACK Button**: Reseta a pose para `(0, 0, 0)` para testes em bancada.

---

## 4. Assistências Autônomas no TeleOp (Padrão Cuberobot #23641)
- **Controlador PIDF de Heading**: `HEADING_LOCK_PIDF = new PIDFCoefficients(1.3, 0.0, 0.05, 0.0)`.
  - Calcula o erro via `MathFunctions.getSmallestAngleDifference` com direção por `MathFunctions.getTurnDirection`.
  - Zona morta de $1^\circ$ para prevenir oscilações em repouso.
  - O PID só reseta na borda de ativação, sem spikes de derivativo.
- **Left Trigger (> 0.2)**: Snaps de 90° dinâmico (`snapToNearestCardinal()`) para esquadrinhar com paredes, submersíveis e portões enquanto segurado.
- **Right Trigger (> 0.2)**: Trava de linha reta na orientação frontal de ciclo da aliança (`fieldHeadingOffset`).
- **D-Pad (Up, Right, Down, Left)**: Snaps instantâneos de 1 toque para $0^\circ, 90^\circ, 180^\circ, -90^\circ$.
- **Right Bumper (⚡ KICK)**: Arranque autônomo guiado por PIDF direto para a pose de pontuação (`CYCLE_1_SCORE`).
  - Cancela se distância < 10", timeout > 0.9s, segundo toque no RB ou se o piloto mexer nos analógicos (> 0.25).
- **Override Manual Instantâneo**: Qualquer movimento no analógico direito de giro destrava o Heading Lock imediatamente e devolve 100% do controle ao piloto.
