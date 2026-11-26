package frc.robot.campo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.arm.ArmKinematics;
import java.util.ArrayList;
import java.util.List;

/**
 * Colisões simplificadas: o robô é um quadrado, o braço é uma fileira de esferas e o
 * Reef é um cilindro (base larga e miolo fino). Não é colisão peça por peça do CAD.
 */
public final class Colisoes {
  private Colisoes() {}

  /** Metade do lado do bumper (34,5 pol). */
  public static final double MEIO_ROBO = 0.438;

  private static final double ALTURA_CHASSI = 0.16;
  private static final double TORRE_MEIO_X = 0.14;
  private static final double TORRE_ALTURA = 0.46;

  private static final double ALTURA_PAREDE = 0.51;
  private static final double REEF_BASE_ALTURA = 0.50;
  private static final double REEF_MIOLO_RAIO = 0.45;
  private static final double REEF_MIOLO_ALTURA = 1.80;

  /** Esfera em um ponto do braço, relativa ao robô. */
  private record Ponto(Translation3d posicao, double raio, boolean ponta) {}

  private static List<Ponto> pontosDoBraco(double ombro, double pontaRel) {
    Pose3d[] poses = ArmKinematics.poses(ombro, pontaRel);
    Translation3d s = poses[0].getTranslation();
    Translation3d e = poses[1].getTranslation();
    Translation3d p = ArmKinematics.ponta(ombro, pontaRel).getTranslation();
    List<Ponto> pontos = new ArrayList<>();
    for (double t = 0.3; t <= 1.001; t += 0.1) {
      pontos.add(new Ponto(s.interpolate(e, t), 0.08, false));
    }
    for (double t = 0.3; t <= 1.001; t += 0.1) {
      pontos.add(new Ponto(e.interpolate(p, t), 0.10, true));
    }
    // A garra é larga: confere também os dois lados.
    for (double lado : new double[] {-0.16, 0.16}) {
      pontos.add(new Ponto(p.plus(new Translation3d(0.0, lado, 0.0)), 0.10, true));
    }
    return pontos;
  }

  /** O 2º estágio não pode entrar no piso, no chassi nem nas torres. */
  public static boolean bracoLivreNoRobo(double ombro, double pontaRel) {
    for (Ponto ponto : pontosDoBraco(ombro, pontaRel)) {
      if (!ponto.ponta()) {
        continue;
      }
      double x = Math.abs(ponto.posicao().getX());
      double baixo = ponto.posicao().getZ() - ponto.raio();
      if (baixo < 0.0
          || (x < MEIO_ROBO && baixo < ALTURA_CHASSI)
          || (x < TORRE_MEIO_X && baixo < TORRE_ALTURA)) {
        return false;
      }
    }
    return true;
  }

  /** O braço não pode entrar na parede, no Reef, na Barge nem nas Cages. */
  public static boolean bracoLivreNoCampo(Pose2d robo, double ombro, double pontaRel) {
    double cos = robo.getRotation().getCos();
    double sin = robo.getRotation().getSin();
    for (Ponto ponto : pontosDoBraco(ombro, pontaRel)) {
      Translation3d p = ponto.posicao();
      double x = robo.getX() + p.getX() * cos - p.getY() * sin;
      double y = robo.getY() + p.getX() * sin + p.getY() * cos;
      if (encosta(x, y, p.getZ(), ponto.raio())) {
        return false;
      }
    }
    return true;
  }

  /** Uma esfera no campo encosta em algum obstáculo? */
  static boolean encosta(double x, double y, double z, double raio) {
    double baixo = z - raio;
    boolean foraDoCampo =
        x < raio || x > Campo.COMPRIMENTO - raio || y < raio || y > Campo.LARGURA - raio;
    if (foraDoCampo && baixo < ALTURA_PAREDE) {
      return true;
    }
    for (double[] reef : Campo.REEFS) {
      double d = Math.hypot(x - reef[0], y - reef[1]);
      if ((baixo < REEF_BASE_ALTURA && d < Campo.REEF_RAIO + raio)
          || (baixo < REEF_MIOLO_ALTURA && d < REEF_MIOLO_RAIO + raio)) {
        return true;
      }
    }
    // Treliça da Barge, no alto.
    if (x > Campo.BARGE_X_MIN - raio
        && x < Campo.BARGE_X_MAX + raio
        && z + raio > Campo.BARGE_TRELICA_ALTURA
        && baixo < Campo.BARGE_TOPO_ALTURA) {
      return true;
    }
    if (baixo < Campo.BARGE_TRELICA_ALTURA) {
      if (x > Campo.SUPORTE_X_MIN - raio
          && x < Campo.SUPORTE_X_MAX + raio
          && y > Campo.SUPORTE_Y_MIN - raio
          && y < Campo.SUPORTE_Y_MAX + raio) {
        return true;
      }
      double meia = Campo.CAGE_LARGURA / 2.0 + raio;
      for (double cageY : Campo.CAGES_Y) {
        if (Math.abs(x - Campo.BARGE_X_CENTRO) < meia && Math.abs(y - cageY) < meia) {
          return true;
        }
      }
    }
    return false;
  }

  /** Empurra o chassi para fora das paredes, Reefs, suporte e Cages. */
  public static Pose2d chassiNoCampo(Pose2d pose) {
    double cos = Math.abs(pose.getRotation().getCos());
    double sin = Math.abs(pose.getRotation().getSin());
    // Quanto o quadrado ocupa em X e em Y do campo (cresce na diagonal).
    double meio = MEIO_ROBO * (cos + sin);
    double x = MathUtil.clamp(pose.getX(), meio, Campo.COMPRIMENTO - meio);
    double y = MathUtil.clamp(pose.getY(), meio, Campo.LARGURA - meio);
    Obstaculos.Batida batida = Obstaculos.empurrarParaFora(x, y, meio, meio, 0.0);
    if (batida != null) {
      x = batida.x;
      y = batida.y;
    }
    return new Pose2d(x, y, pose.getRotation());
  }
}
