package frc.robot.subsystems.arm;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Roda o controlador contra a simulação, ciclo a ciclo, sem abrir o robô. */
class ArmSimTest {
  // As classes de simulação do WPILib precisam do HAL iniciado.
  @BeforeAll
  static void iniciarHal() {
    HAL.initialize(500, 0);
  }

  private final ArmIOSim sim = new ArmIOSim();
  private final ArmController controle = new ArmController();
  private final ArmIOInputs in = new ArmIOInputs();
  private double maiorErroPontaGraus = 0.0;
  private double maiorDesvioOmbroGraus = 0.0;

  private void rodar(double segundos, double pontaEsperadaGraus) {
    for (int i = 0; i < segundos / ArmConstants.PERIODO_S; i++) {
      ArmController.Saida s =
          controle.calcular(
              in.ombroPosicaoRad,
              in.ombroVelocidadeRadS,
              in.eixoPosicaoRad,
              in.eixoVelocidadeRadS);
      sim.setOmbroVolts(s.ombroVolts());
      sim.setEixoVolts(s.eixoVolts());
      sim.updateInputs(in);
      maiorErroPontaGraus =
          Math.max(maiorErroPontaGraus, Math.abs(rel() - pontaEsperadaGraus));
      maiorDesvioOmbroGraus = Math.max(maiorDesvioOmbroGraus, Math.abs(ombro()));
    }
  }

  private double ombro() {
    return radiansToDegrees(in.ombroPosicaoRad);
  }

  private double eixo() {
    return radiansToDegrees(in.eixoPosicaoRad);
  }

  private double rel() {
    return radiansToDegrees(
        ArmKinematics.pontaRel(in.eixoPosicaoRad, in.ombroPosicaoRad));
  }

  @Test
  void posicoesProntas_chegamNoAlvoEmSequencia() {
    ArmIOSim robo =
        new ArmIOSim(
            degreesToRadians(ArmConstants.Posicao.GUARDADO.ombroGraus),
            degreesToRadians(ArmConstants.Posicao.GUARDADO.pontaRelGraus));
    robo.updateInputs(in);
    var ordem =
        new ArmConstants.Posicao[] {
          ArmConstants.Posicao.COLETAR,
          ArmConstants.Posicao.ALTO,
          ArmConstants.Posicao.MEDIO,
          ArmConstants.Posicao.GUARDADO
        };
    for (var p : ordem) {
      controle.compensado(
          degreesToRadians(p.ombroGraus),
          degreesToRadians(p.pontaRelGraus),
          in.ombroPosicaoRad,
          in.eixoPosicaoRad);
      for (int i = 0; i < 250; i++) {
        var s =
            controle.calcular(
                in.ombroPosicaoRad,
                in.ombroVelocidadeRadS,
                in.eixoPosicaoRad,
                in.eixoVelocidadeRadS);
        robo.setOmbroVolts(s.ombroVolts());
        robo.setEixoVolts(s.eixoVolts());
        robo.updateInputs(in);
      }
      assertEquals(p.ombroGraus, ombro(), 0.5, p.name());
      assertEquals(p.pontaRelGraus, rel(), 0.5, p.name());
    }
  }

  @Test
  void somenteA() {
    controle.somenteA(degreesToRadians(30), 0.0, 0.0);
    rodar(3.0, 0.0);
    assertEquals(30.0, ombro(), 0.5);
    assertEquals(0.0, eixo(), 0.5);
    assertEquals(-10.0, rel(), 0.3);
  }

  @Test
  void somenteB() {
    controle.somenteB(degreesToRadians(30), 0.0, 0.0);
    rodar(3.0, 0.0);
    assertEquals(30.0, eixo(), 0.5);
    assertEquals(10.0, rel(), 0.3);
    assertTrue(maiorDesvioOmbroGraus < 0.5, "ombro mexeu " + maiorDesvioOmbroGraus);
  }

  @Test
  void compensado_pontaNaoMexeDuranteOMovimento() {
    controle.compensado(degreesToRadians(60), 0.0, 0.0, 0.0);
    rodar(3.0, 0.0);
    assertEquals(60.0, ombro(), 0.5);
    assertTrue(maiorErroPontaGraus < 1.0, "ponta desviou " + maiorErroPontaGraus);
    System.out.println("compensado: maior desvio da ponta = " + maiorErroPontaGraus);
  }

  @Test
  void compensado_comPontaDobrada_seguraContraGravidade() {
    controle.compensado(degreesToRadians(45), degreesToRadians(30), 0.0, 0.0);
    rodar(4.0, 30.0);
    assertEquals(45.0, ombro(), 0.5);
    assertEquals(30.0, rel(), 0.5);
  }
}
