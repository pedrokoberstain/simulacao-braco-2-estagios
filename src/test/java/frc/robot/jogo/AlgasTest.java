package frc.robot.jogo;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.arm.ArmConstants.Posicao;
import org.junit.jupiter.api.Test;

class AlgasTest {
  private Pose2d pose = new Pose2d(2.0, 2.0, new Rotation2d());
  private double ombro = degreesToRadians(Posicao.COLETAR.ombroGraus);
  private double ponta = degreesToRadians(Posicao.COLETAR.pontaRelGraus);
  private double garra = 0.0;
  private final Algas algas = new Algas(() -> pose, () -> ombro, () -> ponta, () -> garra);

  private void ciclos(int n) {
    for (int i = 0; i < n; i++) {
      algas.periodic();
    }
  }

  @Test
  void coletaArremessaEABolaCaiNoChao() {
    // Põe uma bola solta bem na boca da garra.
    Translation3d boca = algas.posicaoDaGarra();
    Algas.Alga bola = algas.lista().get(0);
    bola.posicao = boca;
    garra = 1.0;
    ciclos(1);
    assertTrue(algas.temAlgaNaGarra());

    // Anda: a bola vai junto.
    pose = new Pose2d(2.5, 2.0, new Rotation2d());
    ciclos(1);
    assertEquals(0.0, bola.posicao.getDistance(algas.posicaoDaGarra()), 1e-9);

    ombro = degreesToRadians(Posicao.ALTO.ombroGraus);
    ponta = degreesToRadians(Posicao.ALTO.pontaRelGraus);
    ciclos(1);
    double alturaSolta = bola.posicao.getZ();
    garra = -1.0;
    ciclos(1);
    garra = 0.0;
    assertTrue(!algas.temAlgaNaGarra());
    ciclos(10);
    assertTrue(bola.posicao.getZ() > alturaSolta, "devia subir no arremesso");
    ciclos(400);
    assertEquals(Algas.RAIO, bola.posicao.getZ(), 0.02);
  }

  @Test
  void arremessoDoAltoPertoDaBargePontuaNaNet() {
    ombro = degreesToRadians(Posicao.ALTO.ombroGraus);
    ponta = degreesToRadians(Posicao.ALTO.pontaRelGraus);
    boolean marcou = false;
    // Procura a distância certa, como um piloto faria.
    for (double x = 5.5; x <= 7.8 && !marcou; x += 0.1) {
      algas.reiniciar();
      garra = 0.0;
      pose = new Pose2d(x, 2.5, new Rotation2d());
      ciclos(40);
      algas.lista().get(0).posicao = algas.posicaoDaGarra();
      garra = 1.0;
      ciclos(1);
      garra = -1.0;
      ciclos(1);
      garra = 0.0;
      ciclos(200);
      if (algas.contar(Algas.Estado.NA_NET) == 1) {
        System.out.println("net a partir de x = " + x);
        marcou = true;
      }
    }
    assertTrue(marcou);
  }
}
