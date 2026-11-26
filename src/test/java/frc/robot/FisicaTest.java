package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.campo.Campo;
import frc.robot.campo.Colisoes;
import frc.robot.subsystems.arm.ArmConstants.Posicao;
import frc.robot.subsystems.arm.ArmController;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmKinematics;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FisicaTest {
  @BeforeAll
  static void iniciarHal() {
    HAL.initialize(500, 0);
  }

  private final ArmController controle = new ArmController();
  private final ArmIOInputs in = new ArmIOInputs();

  private void rodar(ArmIOSim sim, int ciclos) {
    for (int i = 0; i < ciclos; i++) {
      var s =
          controle.calcular(
              in.ombroPosicaoRad,
              in.ombroVelocidadeRadS,
              in.eixoPosicaoRad,
              in.eixoVelocidadeRadS);
      sim.setOmbroVolts(s.ombroVolts());
      sim.setEixoVolts(s.eixoVolts());
      sim.updateInputs(in);
    }
  }

  private double rel() {
    return ArmKinematics.pontaRel(in.eixoPosicaoRad, in.ombroPosicaoRad);
  }

  private ArmIOSim bracoEm(Posicao p) {
    ArmIOSim sim =
        new ArmIOSim(degreesToRadians(p.ombroGraus), degreesToRadians(p.pontaRelGraus));
    sim.updateInputs(in);
    return sim;
  }

  @Test
  void posicoesProntasNaoColidemComORobo() {
    for (Posicao p : Posicao.values()) {
      assertTrue(
          Colisoes.bracoLivreNoRobo(
              degreesToRadians(p.ombroGraus), degreesToRadians(p.pontaRelGraus)),
          p.name());
    }
  }

  @Test
  void todasAsTrocasDePosicaoChegamComColisaoLigada() {
    controle.setPermitido(Colisoes::bracoLivreNoRobo);
    for (Posicao de : Posicao.values()) {
      for (Posicao para : Posicao.values()) {
        ArmIOSim sim = bracoEm(de);
        controle.compensado(
            degreesToRadians(para.ombroGraus),
            degreesToRadians(para.pontaRelGraus),
            in.ombroPosicaoRad,
            in.eixoPosicaoRad);
        rodar(sim, 300);
        assertEquals(
            degreesToRadians(para.pontaRelGraus), rel(), 0.02, de + " -> " + para);
        assertEquals(
            degreesToRadians(para.ombroGraus), in.ombroPosicaoRad, 0.02, de + " -> " + para);
      }
    }
  }

  @Test
  void bracoTravaAntesDeEntrarNoChao() {
    controle.setPermitido(Colisoes::bracoLivreNoRobo);
    ArmIOSim sim = bracoEm(Posicao.COLETAR);
    // Empurra o ombro para a frente até o fim.
    controle.manual(degreesToRadians(-60), 0.0, in.ombroPosicaoRad, in.eixoPosicaoRad);
    rodar(sim, 300);
    assertTrue(in.ombroPosicaoRad > degreesToRadians(-85), "não travou");
    assertTrue(
        ArmKinematics.ponta(in.ombroPosicaoRad, rel()).getZ() > 0.07,
        "ponta " + ArmKinematics.ponta(in.ombroPosicaoRad, rel()).getZ());
  }

  @Test
  void chassiNaoAtravessaParedeNemReef() {
    Pose2d parede = Colisoes.chassiNoCampo(new Pose2d(0.0, 2.0, new Rotation2d()));
    assertEquals(Colisoes.MEIO_ROBO, parede.getX(), 1e-6);

    double[] reef = Campo.REEFS[0];
    Pose2d noReef =
        Colisoes.chassiNoCampo(new Pose2d(reef[0] - 1.0, reef[1], new Rotation2d()));
    assertTrue(reef[0] - noReef.getX() >= Campo.REEF_RAIO + Colisoes.MEIO_ROBO - 1e-6);
  }

  @Test
  void bracoNaoEntraNoReef() {
    double[] reef = Campo.REEFS[0];
    Pose2d longe = new Pose2d(reef[0] - 3.0, reef[1], new Rotation2d());
    Pose2d dentro = new Pose2d(reef[0] - 0.9, reef[1], new Rotation2d());
    double q = degreesToRadians(Posicao.COLETAR.ombroGraus);
    double r = degreesToRadians(Posicao.COLETAR.pontaRelGraus);
    assertTrue(Colisoes.bracoLivreNoCampo(longe, q, r));
    assertFalse(Colisoes.bracoLivreNoCampo(dentro, q, r));
  }
}
