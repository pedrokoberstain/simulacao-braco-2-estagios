package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.campo.Colisoes;
import frc.robot.jogo.Algas;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants.Posicao;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.garra.Garra;
import frc.robot.subsystems.tracao.Tracao;

public class RobotContainer {
  private static final String ALVO_OMBRO = "Braco/Alvo ombro (graus)";
  private static final String ALVO_EIXO = "Braco/Alvo eixo (graus)";
  private static final String ALVO_PONTA = "Braco/Alvo ponta relativa (graus)";

  private static final double ZONA_MORTA_BRACO = 0.2;

  private final Tracao tracao = new Tracao();
  private final Garra garra = new Garra();
  // No robô real ainda não há ArmIO: fica o vazio.
  private final Arm braco =
      new Arm(
          RobotBase.isSimulation()
              ? new ArmIOSim(
                  Units.degreesToRadians(Posicao.GUARDADO.ombroGraus),
                  Units.degreesToRadians(Posicao.GUARDADO.pontaRelGraus))
              : new ArmIO() {},
          tracao::getPose);
  private final CommandXboxController controle = new CommandXboxController(0);
  private final Algas algas =
      new Algas(tracao::getPose, braco::ombroRad, braco::pontaRelRad, garra::getPotencia);

  public RobotContainer() {
    // Na simulação nem sempre há controle plugado; evita avisos repetidos.
    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    // Colisões: o braço trava antes de bater, e o robô não leva o braço contra o campo.
    braco.setPermitido(
        (ombro, ponta) ->
            Colisoes.bracoLivreNoRobo(ombro, ponta)
                && Colisoes.bracoLivreNoCampo(tracao.getPose(), ombro, ponta));
    tracao.setBracoLivre(
        pose -> Colisoes.bracoLivreNoCampo(pose, braco.ombroRad(), braco.pontaRelRad()));
    SmartDashboard.putData(
        "Jogo/Reiniciar bolas",
        Commands.runOnce(algas::reiniciar, algas).ignoringDisable(true));

    // Andar: RT acelera, LT freia e dá ré, analógico esquerdo vira.
    // Segurando o botão do analógico esquerdo, tudo fica em meia velocidade.
    tracao.setDefaultCommand(
        tracao.arcade(
            () -> fator() * (controle.getRightTriggerAxis() - controle.getLeftTriggerAxis()),
            () -> -fator() * controle.getLeftX()));

    // Braço manual no analógico direito: cima/baixo = ombro, lados = cotovelo.
    // Só a direção dominante vale, para não mexer as duas sem querer.
    new Trigger(
            () -> Math.hypot(controle.getRightX(), controle.getRightY()) > ZONA_MORTA_BRACO)
        .whileTrue(
            braco.manual(
                () -> fator() * eixoDominante(controle.getRightY(), controle.getRightX()),
                () -> fator() * eixoDominante(controle.getRightX(), controle.getRightY())));

    // Braço: posições prontas.
    controle.a().onTrue(braco.irPara(Posicao.GUARDADO));
    controle.b().onTrue(braco.irPara(Posicao.COLETAR));
    controle.x().onTrue(braco.irPara(Posicao.MEDIO));
    controle.y().onTrue(braco.irPara(Posicao.ALTO));

    // Garra: LB coleta, RB arremessa.
    controle.leftBumper().whileTrue(garra.rodar(1.0));
    controle.rightBumper().whileTrue(garra.rodar(-1.0));

    // Testes do acoplamento e posições, pelo SmartDashboard.
    SmartDashboard.putNumber(ALVO_OMBRO, 30.0);
    SmartDashboard.putNumber(ALVO_EIXO, 30.0);
    SmartDashboard.putNumber(ALVO_PONTA, 0.0);
    SmartDashboard.putData(
        "Braco/Teste A - so grupo A",
        braco.somenteA(() -> SmartDashboard.getNumber(ALVO_OMBRO, 0.0)));
    SmartDashboard.putData(
        "Braco/Teste B - so grupo B",
        braco.somenteB(() -> SmartDashboard.getNumber(ALVO_EIXO, 0.0)));
    SmartDashboard.putData(
        "Braco/Teste C - compensado",
        braco.compensado(
            () -> SmartDashboard.getNumber(ALVO_OMBRO, 0.0),
            () -> SmartDashboard.getNumber(ALVO_PONTA, 0.0)));
    for (Posicao p : Posicao.values()) {
      SmartDashboard.putData("Braco/" + p.name(), braco.irPara(p));
    }
  }

  private double fator() {
    return controle.getHID().getLeftStickButton() ? 0.5 : 1.0;
  }

  /** Devolve o valor só se este eixo for o mais empurrado dos dois. */
  private static double eixoDominante(double eixo, double outro) {
    return Math.abs(eixo) >= Math.abs(outro)
        ? MathUtil.applyDeadband(eixo, ZONA_MORTA_BRACO)
        : 0.0;
  }

  double ombroGraus() {
    return braco.ombroGraus();
  }

  double pontaRelGraus() {
    return braco.pontaRelGraus();
  }

  Pose2d poseRobo() {
    return tracao.getPose();
  }
}
