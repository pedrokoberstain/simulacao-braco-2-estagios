package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiPredicate;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import frc.robot.subsystems.arm.ArmConstants.Posicao;

import static frc.robot.subsystems.arm.ArmConstants.*;

/** Subsistema do braço de 2 estágios. Os comandos recebem ângulos em graus. */
public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmController controle = new ArmController();

  // Poses dos componentes direto no NetworkTables.
  private final StructArrayPublisher<Pose3d> publicadorPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Arm/PosesComponentes", Pose3d.struct)
          .publish();

  // Onde o robô está no campo (vem da tração).
  private final Supplier<Pose2d> poseRobo;

  // Desenho 2D de reserva (vista lateral: frente do robô à direita).
  private final LoggedMechanism2d desenho = new LoggedMechanism2d(3.0, 2.0);
  private final LoggedMechanismLigament2d desenho1 =
      desenho
          .getRoot("ombro", 1.5, OMBRO_Z)
          .append(
              new LoggedMechanismLigament2d(
                  "estagio1", COMPRIMENTO_1, 90.0, 8.0, new Color8Bit(Color.kOrange)));
  private final LoggedMechanismLigament2d desenho2 =
      desenho1.append(
          new LoggedMechanismLigament2d(
              "estagio2", COMPRIMENTO_2, 0.0, 5.0, new Color8Bit(Color.kLime)));

  public Arm(ArmIO io) {
    this(io, () -> new Pose2d(2.0, 2.0, new Rotation2d()));
  }

  public Arm(ArmIO io, Supplier<Pose2d> poseRobo) {
    this.io = io;
    this.poseRobo = poseRobo;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    io.setFreio(DriverStation.isDisabled());
    if (DriverStation.isDisabled()) {
      // Desabilitado: motores sem tensão e o alvo acompanha a posição real,
      // para o braço não dar um pulo quando habilitar.
      controle.segurar(inputs.ombroPosicaoRad, inputs.eixoPosicaoRad);
      io.setOmbroVolts(0.0);
      io.setEixoVolts(0.0);
    } else {
      ArmController.Saida saida =
          controle.calcular(
              inputs.ombroPosicaoRad,
              inputs.ombroVelocidadeRadS,
              inputs.eixoPosicaoRad,
              inputs.eixoVelocidadeRadS);
      io.setOmbroVolts(saida.ombroVolts());
      io.setEixoVolts(saida.eixoVolts());
    }

    publicar();
  }

  private void publicar() {
    double ombro = inputs.ombroPosicaoRad;
    double eixo = inputs.eixoPosicaoRad;
    double rel = ArmKinematics.pontaRel(eixo, ombro);

    Pose3d[] poses = ArmKinematics.poses(ombro, rel);
    publicadorPoses.set(poses);
    Logger.recordOutput("Arm/PosesComponentes", poses);
    // Poses zeradas: usadas para conferir o encaixe do modelo 3D.
    Logger.recordOutput("Arm/PosesZero", new Pose3d[] {new Pose3d(), new Pose3d()});

    Logger.recordOutput(
        "Arm/EixosNoCampo",
        new Pose3d[] {
          noCampo(poses[0]), noCampo(poses[1]), noCampo(ArmKinematics.ponta(ombro, rel))
        });

    Logger.recordOutput("Arm/OmbroGraus", Units.radiansToDegrees(ombro));
    Logger.recordOutput("Arm/EixoGraus", Units.radiansToDegrees(eixo));
    Logger.recordOutput("Arm/PontaRelGraus", Units.radiansToDegrees(rel));
    Logger.recordOutput(
        "Arm/PontaAbsolutaGraus",
        Units.radiansToDegrees(ArmKinematics.pontaAbsoluta(ombro, rel)));
    Logger.recordOutput("Arm/Modo", controle.getModo());

    desenho1.setAngle(90.0 + Units.radiansToDegrees(ombro));
    desenho2.setAngle(Units.radiansToDegrees(rel));
    Logger.recordOutput("Arm/Mechanism2d", desenho);
  }

  /** Define quais poses (ombro, pontaRel, em radianos) não colidem. */
  public void setPermitido(BiPredicate<Double, Double> permitido) {
    controle.setPermitido(permitido);
  }

  public double ombroRad() {
    return inputs.ombroPosicaoRad;
  }

  public double pontaRelRad() {
    return ArmKinematics.pontaRel(inputs.eixoPosicaoRad, inputs.ombroPosicaoRad);
  }

  public double ombroGraus() {
    return Units.radiansToDegrees(inputs.ombroPosicaoRad);
  }

  public double pontaRelGraus() {
    return Units.radiansToDegrees(
        ArmKinematics.pontaRel(inputs.eixoPosicaoRad, inputs.ombroPosicaoRad));
  }

  /** Converte uma pose relativa ao robô para pose no campo. */
  private Pose3d noCampo(Pose3d relativa) {
    return new Pose3d(poseRobo.get())
        .plus(new Transform3d(relativa.getTranslation(), relativa.getRotation()));
  }

  /** Vai para uma posição pronta, com os dois grupos compensados. */
  public Command irPara(Posicao posicao) {
    return compensado(() -> posicao.ombroGraus, () -> posicao.pontaRelGraus);
  }

  /**
   * Analógico: cada valor vai de -1 a 1. Ombro positivo inclina para trás; cotovelo
   * positivo abre a ponta. Ao soltar, para e segura.
   */
  public Command manual(DoubleSupplier ombro, DoubleSupplier cotovelo) {
    return runEnd(
        () ->
            controle.manual(
                ombro.getAsDouble() * OMBRO_VEL_MANUAL,
                cotovelo.getAsDouble() * PONTA_VEL_MANUAL,
                inputs.ombroPosicaoRad,
                inputs.eixoPosicaoRad),
        () -> controle.manual(0.0, 0.0, inputs.ombroPosicaoRad, inputs.eixoPosicaoRad));
  }

  /** (a) Só o grupo A: leva o ombro ao alvo, grupo B segura o eixo. */
  public Command somenteA(DoubleSupplier ombroGraus) {
    return runOnce(
        () ->
            controle.somenteA(
                Units.degreesToRadians(ombroGraus.getAsDouble()),
                inputs.ombroPosicaoRad,
                inputs.eixoPosicaoRad));
  }

  /** (b) Só o grupo B: leva o eixo ao alvo, grupo A segura o braço. */
  public Command somenteB(DoubleSupplier eixoGraus) {
    return runOnce(
        () ->
            controle.somenteB(
                Units.degreesToRadians(eixoGraus.getAsDouble()),
                inputs.ombroPosicaoRad,
                inputs.eixoPosicaoRad));
  }

  /** (c) Compensado: ombro e ponta relativa com alvos independentes. */
  public Command compensado(DoubleSupplier ombroGraus, DoubleSupplier pontaRelGraus) {
    return runOnce(
        () ->
            controle.compensado(
                Units.degreesToRadians(ombroGraus.getAsDouble()),
                Units.degreesToRadians(pontaRelGraus.getAsDouble()),
                inputs.ombroPosicaoRad,
                inputs.eixoPosicaoRad));
  }
}
