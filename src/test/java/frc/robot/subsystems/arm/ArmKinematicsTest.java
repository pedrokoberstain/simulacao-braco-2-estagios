package frc.robot.subsystems.arm;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.Test;

class ArmKinematicsTest {
  private static final double TOL = 1e-9;

  @Test
  void somenteGrupoA_ombroMais30_pontaMenos10() {
    double rel = ArmKinematics.pontaRel(0.0, degreesToRadians(30));
    assertEquals(-10.0, radiansToDegrees(rel), TOL);
  }

  @Test
  void somenteGrupoB_eixoMais30_pontaMais10() {
    double rel = ArmKinematics.pontaRel(degreesToRadians(30), 0.0);
    assertEquals(10.0, radiansToDegrees(rel), TOL);
  }

  @Test
  void compensado_pontaRelConstante() {
    double alvoRel = degreesToRadians(25);
    for (int graus = -90; graus <= 90; graus += 5) {
      double ombro = degreesToRadians(graus);
      double eixo = ArmKinematics.eixoPara(ombro, alvoRel);
      assertEquals(ombro + 3 * alvoRel, eixo, TOL);
      assertEquals(alvoRel, ArmKinematics.pontaRel(eixo, ombro), TOL);
    }
  }

  @Test
  void posesNoZero_bracoEsticadoParaCima() {
    Pose3d cotovelo = ArmKinematics.poses(0.0, 0.0)[1];
    assertEquals(0.0, cotovelo.getX(), TOL);
    assertEquals(ArmConstants.OMBRO_Z + ArmConstants.COMPRIMENTO_1, cotovelo.getZ(), TOL);
  }

  @Test
  void anguloPositivoInclinaParaTras() {
    Pose3d[] poses = ArmKinematics.poses(degreesToRadians(90), 0.0);
    assertEquals(-ArmConstants.COMPRIMENTO_1, poses[1].getX(), TOL);
    assertEquals(ArmConstants.OMBRO_Z, poses[1].getZ(), TOL);
    // A rotação publicada leva o eixo "para cima" na mesma direção.
    assertEquals(-degreesToRadians(90), poses[0].getRotation().getY(), 1e-6);
  }
}
