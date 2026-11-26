package frc.robot.jogo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.campo.Campo;
import frc.robot.campo.Colisoes;
import frc.robot.campo.Obstaculos;
import frc.robot.subsystems.arm.ArmKinematics;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Bolas (Algae) do jogo de 2025: paradas no Reef e nos suportes, coletadas pela garra,
 * arremessadas, quicando no campo e pontuando na Net e no Processor.
 */
public class Algas extends SubsystemBase {
  public static final double RAIO = 0.20;
  private static final double DT = 0.02;
  private static final double GRAVIDADE = 9.81;
  private static final double ALCANCE_GARRA = 0.35;
  private static final double VELOCIDADE_ARREMESSO = 4.5;
  private static final double QUIQUE = 0.5;
  private static final double ATRITO = 1.2;

  // Posições lidas do modelo 3D do campo: {x, y, z}.
  private static final double[][] INICIAIS = {
    // Reef azul
    {5.169, 4.020, 0.909}, {4.829, 3.433, 1.312}, {4.150, 3.433, 0.909},
    {3.810, 4.022, 1.313}, {4.150, 4.609, 0.909}, {4.828, 4.610, 1.313},
    // Reef vermelho
    {13.738, 4.020, 1.313}, {13.398, 3.433, 0.909}, {12.720, 3.432, 1.313},
    {12.379, 4.022, 0.909}, {12.719, 4.609, 1.312}, {13.398, 4.609, 0.909},
    // Suportes perto das alianças
    {1.219, 2.192, 0.496}, {1.219, 4.021, 0.496}, {1.219, 5.850, 0.496},
    {16.329, 2.192, 0.496}, {16.329, 4.021, 0.496}, {16.329, 5.850, 0.496},
  };

  /** PARADA = no Reef ou suporte; SOLTA = obedece à física. */
  enum Estado {
    PARADA,
    SOLTA,
    NA_GARRA,
    NA_NET,
    NO_PROCESSOR
  }

  static final class Alga {
    Translation3d posicao;
    Translation3d velocidade = new Translation3d();
    Estado estado;

    Alga(Translation3d posicao, Estado estado) {
      this.posicao = posicao;
      this.estado = estado;
    }
  }

  private final Supplier<Pose2d> poseRobo;
  private final DoubleSupplier ombroRad;
  private final DoubleSupplier pontaRelRad;
  private final DoubleSupplier garra;
  private final List<Alga> algas = new ArrayList<>();
  private Pose2d poseAnterior;
  private Alga naGarra;
  // Depois de soltar, espera a bola se afastar antes de coletar de novo.
  private double espera = 0.0;

  public Algas(
      Supplier<Pose2d> poseRobo,
      DoubleSupplier ombroRad,
      DoubleSupplier pontaRelRad,
      DoubleSupplier garra) {
    this.poseRobo = poseRobo;
    this.ombroRad = ombroRad;
    this.pontaRelRad = pontaRelRad;
    this.garra = garra;
    reiniciar();
  }

  /** Volta todas as bolas para o começo da partida. */
  public final void reiniciar() {
    algas.clear();
    naGarra = null;
    for (double[] p : INICIAIS) {
      algas.add(new Alga(new Translation3d(p[0], p[1], p[2]), Estado.PARADA));
    }
  }

  /** Onde a bola fica quando está na garra, no campo. */
  public Translation3d posicaoDaGarra() {
    Pose3d ponta = ArmKinematics.ponta(ombroRad.getAsDouble(), pontaRelRad.getAsDouble());
    Translation3d local = ponta.getTranslation().plus(direcaoDaPonta().times(0.12));
    return noCampo(local, poseRobo.get());
  }

  /** Direção do 2º estágio, relativa ao robô. */
  private Translation3d direcaoDaPonta() {
    double angulo =
        ArmKinematics.pontaAbsoluta(ombroRad.getAsDouble(), pontaRelRad.getAsDouble());
    return new Translation3d(-Math.sin(angulo), 0.0, Math.cos(angulo));
  }

  private static Translation3d noCampo(Translation3d local, Pose2d robo) {
    Translation2d xy = local.toTranslation2d().rotateBy(robo.getRotation());
    return new Translation3d(
        robo.getX() + xy.getX(), robo.getY() + xy.getY(), local.getZ());
  }

  public boolean temAlgaNaGarra() {
    return naGarra != null;
  }

  int contar(Estado estado) {
    return (int) algas.stream().filter(a -> a.estado == estado).count();
  }

  List<Alga> lista() {
    return algas;
  }

  @Override
  public void periodic() {
    Pose2d robo = poseRobo.get();
    Translation2d velRobo =
        poseAnterior == null
            ? new Translation2d()
            : robo.getTranslation().minus(poseAnterior.getTranslation()).div(DT);
    poseAnterior = robo;
    espera = Math.max(0.0, espera - DT);

    double potencia = garra.getAsDouble();
    if (naGarra != null && potencia < -0.1) {
      Translation3d direcao =
          noCampo(direcaoDaPonta(), new Pose2d(0.0, 0.0, robo.getRotation()));
      naGarra.velocidade =
          direcao
              .times(VELOCIDADE_ARREMESSO)
              .plus(new Translation3d(velRobo.getX(), velRobo.getY(), 0.0));
      naGarra.estado = Estado.SOLTA;
      naGarra = null;
      espera = 0.6;
    } else if (naGarra == null && potencia > 0.1 && espera == 0.0) {
      coletar();
    }

    for (Alga alga : algas) {
      if (alga.estado == Estado.NA_GARRA) {
        alga.posicao = posicaoDaGarra();
      } else if (alga.estado == Estado.SOLTA) {
        mover(alga);
        empurrarComRobo(alga, robo, velRobo);
      }
    }
    colidirEntreSi();
    publicar();
  }

  private void coletar() {
    Translation3d boca = posicaoDaGarra();
    Alga melhor = null;
    double menor = ALCANCE_GARRA;
    for (Alga alga : algas) {
      double d = alga.posicao.getDistance(boca);
      if ((alga.estado == Estado.PARADA || alga.estado == Estado.SOLTA) && d < menor) {
        melhor = alga;
        menor = d;
      }
    }
    if (melhor != null) {
      melhor.estado = Estado.NA_GARRA;
      melhor.velocidade = new Translation3d();
      naGarra = melhor;
    }
  }

  private void mover(Alga alga) {
    double x = alga.posicao.getX();
    double y = alga.posicao.getY();
    double z = alga.posicao.getZ();
    double vx = alga.velocidade.getX();
    double vy = alga.velocidade.getY();
    double vz = alga.velocidade.getZ() - GRAVIDADE * DT;
    double zAntes = z;
    x += vx * DT;
    y += vy * DT;
    z += vz * DT;

    // Net: caiu de cima para baixo sobre a Barge.
    if (vz < 0
        && zAntes >= Campo.NET_ALTURA
        && z < Campo.NET_ALTURA
        && x > Campo.BARGE_X_MIN
        && x < Campo.BARGE_X_MAX) {
      alga.estado = Estado.NA_NET;
      alga.posicao = new Translation3d(x, y, Campo.NET_ALTURA + RAIO);
      return;
    }

    // Chão: quica e perde velocidade rolando.
    if (z < RAIO) {
      z = RAIO;
      vz = vz < -0.5 ? -vz * QUIQUE : 0.0;
      double v = Math.hypot(vx, vy);
      double nova = Math.max(0.0, v - ATRITO * DT);
      if (v > 0) {
        vx *= nova / v;
        vy *= nova / v;
      }
    }

    // Processor: abertura nas paredes laterais.
    if ((naAbertura(x, z, Campo.PROCESSOR_AZUL_X) && y < RAIO)
        || (naAbertura(x, z, Campo.PROCESSOR_VERMELHO_X) && y > Campo.LARGURA - RAIO)) {
      alga.estado = Estado.NO_PROCESSOR;
      double fora = y < RAIO ? -0.45 : Campo.LARGURA + 0.45;
      alga.posicao = new Translation3d(x, fora, RAIO);
      return;
    }

    // Paredes.
    if (x < RAIO || x > Campo.COMPRIMENTO - RAIO) {
      x = MathUtil.clamp(x, RAIO, Campo.COMPRIMENTO - RAIO);
      vx = -vx * QUIQUE;
    }
    if (y < RAIO || y > Campo.LARGURA - RAIO) {
      y = MathUtil.clamp(y, RAIO, Campo.LARGURA - RAIO);
      vy = -vy * QUIQUE;
    }

    // Reefs, suporte e Cages (só bola baixa).
    if (z < 0.60) {
      Obstaculos.Batida b = Obstaculos.empurrarParaFora(x, y, 0.0, 0.0, RAIO);
      if (b != null) {
        x = b.x;
        y = b.y;
        double contra = vx * b.normalX + vy * b.normalY;
        if (contra < 0) {
          vx -= (1 + QUIQUE) * contra * b.normalX;
          vy -= (1 + QUIQUE) * contra * b.normalY;
        }
      }
    }

    alga.posicao = new Translation3d(x, y, z);
    alga.velocidade = new Translation3d(vx, vy, vz);
  }

  private static boolean naAbertura(double x, double z, double centroX) {
    return Math.abs(x - centroX) < Campo.PROCESSOR_LARGURA / 2.0
        && z >= Campo.PROCESSOR_ALTURA_MIN
        && z <= Campo.PROCESSOR_ALTURA_MAX;
  }

  /** O bumper empurra a bola que está baixa. */
  private void empurrarComRobo(Alga alga, Pose2d robo, Translation2d velRobo) {
    if (alga.estado != Estado.SOLTA || alga.posicao.getZ() > 0.40) {
      return;
    }
    Translation2d local =
        alga.posicao
            .toTranslation2d()
            .minus(robo.getTranslation())
            .rotateBy(robo.getRotation().unaryMinus());
    double m = Colisoes.MEIO_ROBO;
    Translation2d perto =
        new Translation2d(
            MathUtil.clamp(local.getX(), -m, m), MathUtil.clamp(local.getY(), -m, m));
    double d = local.getDistance(perto);
    if (d >= RAIO || d < 1e-6) {
      return;
    }
    Translation2d normal = local.minus(perto).div(d).rotateBy(robo.getRotation());
    double aproximacao = velRobo.getX() * normal.getX() + velRobo.getY() * normal.getY();
    Translation2d empurrada =
        alga.posicao.toTranslation2d().plus(normal.times(RAIO - d));
    alga.posicao =
        new Translation3d(empurrada.getX(), empurrada.getY(), alga.posicao.getZ());
    if (aproximacao > 0) {
      alga.velocidade =
          alga.velocidade.plus(
              new Translation3d(
                  normal.getX() * aproximacao, normal.getY() * aproximacao, 0.0));
    }
  }

  /** Bolas soltas não se atravessam. */
  private void colidirEntreSi() {
    for (int i = 0; i < algas.size(); i++) {
      for (int j = i + 1; j < algas.size(); j++) {
        Alga a = algas.get(i);
        Alga b = algas.get(j);
        if (a.estado != Estado.SOLTA || b.estado != Estado.SOLTA) {
          continue;
        }
        Translation3d entre = b.posicao.minus(a.posicao);
        double d = entre.getNorm();
        if (d >= 2 * RAIO || d < 1e-6) {
          continue;
        }
        Translation3d n = entre.div(d);
        Translation3d ajuste = n.times((2 * RAIO - d) / 2.0);
        a.posicao = a.posicao.minus(ajuste);
        b.posicao = b.posicao.plus(ajuste);
        Translation3d rel = b.velocidade.minus(a.velocidade);
        double contra =
            rel.getX() * n.getX() + rel.getY() * n.getY() + rel.getZ() * n.getZ();
        if (contra < 0) {
          Translation3d impulso = n.times(contra * 0.9);
          a.velocidade = a.velocidade.plus(impulso);
          b.velocidade = b.velocidade.minus(impulso);
        }
      }
    }
  }

  private void publicar() {
    Logger.recordOutput(
        "Jogo/Algas",
        algas.stream()
            .map(a -> new Pose3d(a.posicao, new Rotation3d()))
            .toArray(Pose3d[]::new));
    Logger.recordOutput("Jogo/AlgaNaGarra", naGarra != null);
    Logger.recordOutput("Jogo/NaNet", contar(Estado.NA_NET));
    Logger.recordOutput("Jogo/NoProcessor", contar(Estado.NO_PROCESSOR));
  }
}
