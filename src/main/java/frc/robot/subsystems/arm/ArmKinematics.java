package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.subsystems.arm.ArmConstants.*;

/** Contas de geometria do braço. Todos os ângulos em radianos. */
public final class ArmKinematics {
  private ArmKinematics() {}

  /** Acoplamento da corrente: pontaRel = (eixo − ombro) × 16/48. */
  public static double pontaRel(double eixo, double ombro) {
    return (eixo - ombro) * CORRENTE_PONTA;
  }

  /** Inverso: onde o eixo precisa estar para a ponta ficar em pontaRel. */
  public static double eixoPara(double ombro, double pontaRel) {
    return ombro + pontaRel / CORRENTE_PONTA;
  }

  /** Ângulo da ponta em relação à vertical. */
  public static double pontaAbsoluta(double ombro, double pontaRel) {
    return ombro + pontaRel;
  }

  /**
   * Poses dos componentes em relação ao robô: [0] = 1º estágio (pivô no ombro),
   * [1] = 2º estágio (pivô no cotovelo).
   *
   * <p>No WPILib, pitch positivo inclina o que aponta para cima na direção da
   * frente (+X). Como o nosso ângulo positivo inclina para trás, o pitch é o
   * negativo dele.
   */
  public static Pose3d[] poses(double ombro, double pontaRel) {
    Translation3d posOmbro = new Translation3d(OMBRO_X, OMBRO_Y, OMBRO_Z);
    Pose3d estagio1 = new Pose3d(posOmbro, new Rotation3d(0.0, -ombro, 0.0));
    Pose3d estagio2 =
        new Pose3d(
            extremidade(posOmbro, ombro, COMPRIMENTO_1),
            new Rotation3d(0.0, -pontaAbsoluta(ombro, pontaRel), 0.0));
    return new Pose3d[] {estagio1, estagio2};
  }

  /** Pose da ponta do 2º estágio (só para visualização). */
  public static Pose3d ponta(double ombro, double pontaRel) {
    Pose3d estagio2 = poses(ombro, pontaRel)[1];
    double angulo = pontaAbsoluta(ombro, pontaRel);
    return new Pose3d(
        extremidade(estagio2.getTranslation(), angulo, COMPRIMENTO_2), estagio2.getRotation());
  }

  private static Translation3d extremidade(Translation3d pivo, double angulo, double comprimento) {
    return pivo.plus(
        new Translation3d(
            -comprimento * Math.sin(angulo), 0.0, comprimento * Math.cos(angulo)));
  }
}
