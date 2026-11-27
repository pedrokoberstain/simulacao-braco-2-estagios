# Simulação do Braço de 2 Estágios

![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-blue)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-26.0.2-orange)
![Java](https://img.shields.io/badge/Java-17-red)
![FRC](https://img.shields.io/badge/FRC-%2310297-green)

Simulação 3D de um robô **tank** com **braço de 2 estágios** e garra de rodas, da equipe
**PantanalBots #10297** (SENAI Cáceres/MT). O código roda no simulador do WPILib e é visualizado no
AdvantageScope, no campo de 2025, para testar o mecanismo antes de construí-lo.

> Projeto de simulação: ainda não há código para o hardware real, e parte dos valores físicos é
> provisória (veja [Limitações](#limitações-e-próximos-passos)).

## Sumário

- [Funcionalidades](#funcionalidades)
- [Como executar](#como-executar)
- [Controles](#controles)
- [Modelo 3D e AdvantageScope](#modelo-3d-e-advantagescope)
- [O mecanismo](#o-mecanismo)
- [Física da simulação](#física-da-simulação)
- [Estrutura do projeto](#estrutura-do-projeto)
- [Testes](#testes)
- [Limitações e próximos passos](#limitações-e-próximos-passos)
- [Créditos](#créditos)

## Funcionalidades

- Braço com ombro e cotovelo **independentes**, acoplados por corrente e compensados em software.
- Controle de posição com PID, feedforward e perfil trapezoidal.
- Posições prontas (guardado, coletar, médio, alto) e ajuste manual pelo analógico.
- Tração tank com 4 motores CIM.
- Colisões do chassi e do braço com o robô e com o campo.
- 18 bolas (Algae) para coletar, arremessar e pontuar na Net e no Processor.
- Logs com AdvantageKit (camada IO no braço).

## Como executar

**Requisitos:** [WPILib 2026.2.1](https://docs.wpilib.org) (inclui Java 17 e AdvantageScope) e um
controle Xbox.

```bash
git clone https://github.com/pedrokoberstain/simulacao-braco-2-estagios.git
cd simulacao-braco-2-estagios
./gradlew build          # compila e roda os testes
./gradlew simulateJava   # abre a simulação
```

Na janela da simulação:

1. Coloque o controle em **Joystick[0]**, com o mapeamento de gamepad ligado.
2. Em **Robot State**, escolha **Teleoperated**.

## Controles

| Controle Xbox | Função |
|---|---|
| RT | acelerar |
| LT | frear e dar ré |
| Analógico esquerdo (lados) | virar |
| Analógico direito (cima/baixo) | ombro |
| Analógico direito (lados) | cotovelo |
| A / B / X / Y | guardado / coletar / médio / alto |
| LB (segurado) | coletar bola |
| RB | arremessar |
| Botão do analógico esquerdo (segurado) | meia velocidade |

Em **NetworkTables > SmartDashboard** ficam os testes do acoplamento, os alvos em graus e o botão
**Jogo/Reiniciar bolas**.

## Modelo 3D e AdvantageScope

### Instalar o modelo (recomendado)

1. No Windows, aperte **Win + R** e abra `%APPDATA%\AdvantageScope\userAssets`
   (se a pasta não existir, abra o AdvantageScope uma vez).
2. Extraia [`modelo3d/Robot_Braco2Estagios.zip`](modelo3d/Robot_Braco2Estagios.zip) dentro dela.
   Deve ficar `userAssets\Robot_Braco2Estagios\config.json`, junto dos três arquivos `.glb`.
3. Feche e abra o AdvantageScope.

### Visualizar

1. Com a simulação aberta, use **File > Connect to Simulator**.
2. Use **File > Import Layout** e escolha `advantagescope/layout.json`.

### Gerar o modelo de novo (opcional)

Necessário só se o CAD mudar:

1. Abra o CAD do braço (veja [Créditos](#créditos)) no Onshape e exporte a aba
   **Full W/ (2023)End effecter** como **GLB**, com o eixo Z para cima.
2. Rode o script (requer Python 3 com `numpy`):

   ```bash
   python tools/separar_glb.py "caminho/do/arquivo.glb"
   ```

   Ele grava os arquivos direto na pasta de modelos do AdvantageScope, troca os módulos swerve do
   CAD por rodas de tank e alinha cada estágio no seu pivô.
3. Compacte a pasta gerada para atualizar o zip em `modelo3d/`.

## O mecanismo

| Grupo | Motores | Redução | Move |
|---|---|---|---|
| A (ombro) | 2 × Kraken X60 | 16→60, 16→60, corrente 16→48 = **42,1875:1** | 1º estágio em relação à torre |
| B (eixo) | 2 × Kraken X60 | 16→60, 16→60 = **14,0625:1** | eixo do ombro, que move o cotovelo por corrente 16→48 |

Como a corrente do cotovelo parte do eixo do ombro, as articulações são acopladas:

```
pontaRel   = (anguloEixo - anguloOmbro) × 16/48
alvoEixo   = alvoOmbro + 3 × alvoPontaRel
```

**Convenção:** zero é o braço esticado para cima, e ângulo positivo inclina para trás.

| Medida (do CAD) | Valor |
|---|---|
| Altura do ombro | 15,7 pol (40 cm) |
| 1º estágio (ombro → cotovelo) | 27,74 pol (70 cm) |
| 2º estágio (cotovelo → rodas da garra) | 37,58 pol (95 cm) |

## Física da simulação

- **Braço:** `SingleJointedArmSim` no ombro e motor com carga de gravidade no eixo do cotovelo.
- **Chassi:** não atravessa paredes, Reefs, suporte da Barge nem Cages.
- **Braço contra obstáculos:** trava antes de entrar no piso, no chassi, no Reef, na parede ou na
  treliça da Barge; o robô não anda se isso levaria o braço contra o campo.
- **Bolas:** caem, quicam, rolam, colidem entre si e são empurradas pelo bumper.

O modelo é simplificado: robô quadrado, braço representado por esferas e Reef por cilindros.

## Estrutura do projeto

```
src/main/java/frc/robot
├── RobotContainer.java     controles e ligações entre subsistemas
├── subsystems
│   ├── arm                 constantes, cinemática, controle, IO e simulação do braço
│   ├── garra               garra de rodas
│   └── tracao              tração tank
├── campo                   medidas do campo e colisões
└── jogo                    bolas e pontuação
src/test/java/frc/robot     testes JUnit
tools                       geração do modelo 3D e do layout
advantagescope              layout das abas
modelo3d                    modelo 3D pronto para instalar
```

## Testes

```bash
./gradlew test
```

Os testes cobrem a cinemática do acoplamento, o controle do braço na simulação, as posições
prontas, as colisões, a coleta e o arremesso de bolas e os controles do robô completo.

## Limitações e próximos passos

- Massas, inércias, limites, IDs CAN e medidas da tração estão marcados com `PREENCHER`.
- Motores e reduções são os do CAD; está em estudo usar **NEO** e **MAXPlanetary**.
- O chassi exibido é o do CAD, não o chassi real da equipe.
- As rodas da garra não giram na visualização.
- Falta o `ArmIO` para o hardware real.

## Créditos

- CAD do braço:
  [FRC configurable Arm Robot](https://www.printables.com/model/1123646-frc-configurable-arm-robot),
  publicado no Printables em 28/12/2024 em domínio público (Creative Commons). O modelo em
  `modelo3d/` é uma adaptação dele.
- `campo/Campo.java` e `campo/Obstaculos.java` vêm da simulação 3D da equipe.
- Base do projeto e `WPILib-License.md`: [WPILib](https://github.com/wpilibsuite/allwpilib).

---

**PantanalBots #10297** · SENAI Cáceres/MT
