package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.BiPredicate;

import static frc.robot.subsystems.arm.ArmConstants.*;

/**
 * Controle de posição (PID + feedforward) dos dois grupos. Não depende de
 * hardware, por isso dá para testar com JUnit.
 */
public class ArmController {
  public enum Modo {
    /** Os dois grupos seguram a posição. */
    SEGURAR,
    /** Só o grupo A se move; o grupo B segura o eixo. */
    SOMENTE_A,
    /** Só o grupo B se move; o grupo A segura o braço. */
    SOMENTE_B,
    /** O eixo acompanha o ombro para a ponta obedecer só ao próprio alvo. */
    COMPENSADO,
    /** Analógico: velocidade do ombro e da ponta separadas, também compensado. */
    MANUAL
  }

  /** Tensões calculadas para cada grupo. */
  public record Saida(double ombroVolts, double eixoVolts) {}

  private final PIDController pidOmbro = new PIDController(OMBRO_KP, 0.0, OMBRO_KD, PERIODO_S);
  private final PIDController pidEixo = new PIDController(EIXO_KP, 0.0, EIXO_KD, PERIODO_S);

  // kG: volts para segurar o braço na horizontal. kV: volts por rad/s.
  private final ArmFeedforward ffOmbro =
      new ArmFeedforward(
          0.0,
          MOMENTO_OMBRO
              * GRAVIDADE
              * MOTORES_OMBRO.rOhms
              / (REDUCAO_OMBRO * MOTORES_OMBRO.KtNMPerAmp),
          REDUCAO_OMBRO / MOTORES_OMBRO.KvRadPerSecPerVolt);
  private static final double EIXO_KV = REDUCAO_EIXO / MOTORES_EIXO.KvRadPerSecPerVolt;

  private final TrapezoidProfile perfilOmbro =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(OMBRO_VEL_MAX, OMBRO_ACEL_MAX));
  private final TrapezoidProfile perfilEixo =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(EIXO_VEL_MAX, EIXO_ACEL_MAX));
  private final TrapezoidProfile perfilPonta =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(PONTA_VEL_MAX, PONTA_ACEL_MAX));

  private Modo modo = Modo.SEGURAR;

  // "ref" = ponto do perfil neste instante; "alvo" = destino final.
  private TrapezoidProfile.State refOmbro = new TrapezoidProfile.State();
  private TrapezoidProfile.State refEixo = new TrapezoidProfile.State();
  private TrapezoidProfile.State refPonta = new TrapezoidProfile.State();
  private double alvoOmbro = 0.0;
  private double alvoEixo = 0.0;
  private double alvoPonta = 0.0;

  // Diz se uma pose (ombro, pontaRel) é permitida, isto é, sem colisão.
  private BiPredicate<Double, Double> permitido = (ombro, ponta) -> true;

  public void setPermitido(BiPredicate<Double, Double> permitido) {
    this.permitido = permitido;
  }

  /**
   * Se o próximo passo bater em algo, segura a articulação culpada e deixa a outra
   * continuar. Se a pose já estava em colisão, não trava (para conseguir sair).
   */
  private void barrarColisao(
      TrapezoidProfile.State ombroAntes, TrapezoidProfile.State pontaAntes) {
    if (permitido.test(refOmbro.position, refPonta.position)
        || !permitido.test(ombroAntes.position, pontaAntes.position)) {
      return;
    }
    TrapezoidProfile.State ombroParado =
        new TrapezoidProfile.State(ombroAntes.position, 0.0);
    TrapezoidProfile.State pontaParada =
        new TrapezoidProfile.State(pontaAntes.position, 0.0);
    if (permitido.test(ombroAntes.position, refPonta.position)) {
      refOmbro = ombroParado;
    } else if (permitido.test(refOmbro.position, pontaAntes.position)) {
      refPonta = pontaParada;
    } else {
      refOmbro = ombroParado;
      refPonta = pontaParada;
    }
    alvoOmbro = modo == Modo.MANUAL ? refOmbro.position : alvoOmbro;
  }

  private double velOmbroPedida = 0.0;
  private double velPontaPedida = 0.0;

  /** Move cada articulação na velocidade pedida (rad/s). Zero = parar e segurar. */
  public void manual(double velOmbro, double velPonta, double ombro, double eixo) {
    if (modo != Modo.MANUAL) {
      // Vindo de uma posição pronta, continua de onde a referência está.
      if (modo != Modo.COMPENSADO) {
        refOmbro = new TrapezoidProfile.State(ombro, 0.0);
        refPonta = new TrapezoidProfile.State(ArmKinematics.pontaRel(eixo, ombro), 0.0);
      }
      modo = Modo.MANUAL;
    }
    velOmbroPedida = velOmbro;
    velPontaPedida = velPonta;
  }

  /** Avança a referência: acelera até a velocidade pedida e respeita os limites. */
  private static TrapezoidProfile.State avancar(
      TrapezoidProfile.State ref, double velPedida, double acel, double min, double max) {
    double passo = acel * PERIODO_S;
    double vel = ref.velocity + MathUtil.clamp(velPedida - ref.velocity, -passo, passo);
    double pos = ref.position + vel * PERIODO_S;
    if (pos <= min || pos >= max) {
      return new TrapezoidProfile.State(MathUtil.clamp(pos, min, max), 0.0);
    }
    return new TrapezoidProfile.State(pos, vel);
  }

  public Modo getModo() {
    return modo;
  }

  /** Segura onde está agora. */
  public void segurar(double ombro, double eixo) {
    modo = Modo.SEGURAR;
    fixarOmbro(ombro);
    fixarEixo(eixo);
  }

  public void somenteA(double novoAlvoOmbro, double ombro, double eixo) {
    modo = Modo.SOMENTE_A;
    refOmbro = new TrapezoidProfile.State(ombro, 0.0);
    alvoOmbro = MathUtil.clamp(novoAlvoOmbro, OMBRO_MIN, OMBRO_MAX);
    fixarEixo(eixo);
  }

  public void somenteB(double novoAlvoEixo, double ombro, double eixo) {
    modo = Modo.SOMENTE_B;
    fixarOmbro(ombro);
    refEixo = new TrapezoidProfile.State(eixo, 0.0);
    // O limite vale para a ponta, então depende de onde o ombro está parado.
    alvoEixo =
        MathUtil.clamp(
            novoAlvoEixo,
            ArmKinematics.eixoPara(ombro, PONTA_MIN),
            ArmKinematics.eixoPara(ombro, PONTA_MAX));
  }

  public void compensado(
      double novoAlvoOmbro, double novoAlvoPonta, double ombro, double eixo) {
    modo = Modo.COMPENSADO;
    refOmbro = new TrapezoidProfile.State(ombro, 0.0);
    refPonta = new TrapezoidProfile.State(ArmKinematics.pontaRel(eixo, ombro), 0.0);
    alvoOmbro = MathUtil.clamp(novoAlvoOmbro, OMBRO_MIN, OMBRO_MAX);
    alvoPonta = MathUtil.clamp(novoAlvoPonta, PONTA_MIN, PONTA_MAX);
  }

  private void fixarOmbro(double ombro) {
    refOmbro = new TrapezoidProfile.State(ombro, 0.0);
    alvoOmbro = ombro;
  }

  private void fixarEixo(double eixo) {
    refEixo = new TrapezoidProfile.State(eixo, 0.0);
    alvoEixo = eixo;
  }

  /** Chamar uma vez a cada ciclo de 20 ms. */
  public Saida calcular(double ombro, double ombroVel, double eixo, double eixoVel) {
    TrapezoidProfile.State ombroAntes = refOmbro;
    TrapezoidProfile.State pontaAntes = refPonta;

    if (modo == Modo.MANUAL) {
      refOmbro =
          avancar(refOmbro, velOmbroPedida, OMBRO_ACEL_MAX, OMBRO_MIN, OMBRO_MAX);
      refPonta =
          avancar(refPonta, velPontaPedida, PONTA_ACEL_MAX, PONTA_MIN, PONTA_MAX);
      alvoOmbro = refOmbro.position;
    } else {
      refOmbro =
          perfilOmbro.calculate(PERIODO_S, refOmbro, new TrapezoidProfile.State(alvoOmbro, 0.0));
    }

    double eixoPos;
    double eixoVelRef;
    if (modo == Modo.COMPENSADO || modo == Modo.MANUAL) {
      if (modo == Modo.COMPENSADO) {
        refPonta =
            perfilPonta.calculate(
                PERIODO_S, refPonta, new TrapezoidProfile.State(alvoPonta, 0.0));
      }
      barrarColisao(ombroAntes, pontaAntes);
      // eixo = ombro + 3 × pontaRel. Usa o ombro MEDIDO (previsto um ciclo à
      // frente), assim a ponta não se mexe mesmo se o ombro atrasar.
      eixoPos =
          ArmKinematics.eixoPara(ombro + ombroVel * PERIODO_S, refPonta.position);
      eixoVelRef = ombroVel + refPonta.velocity / CORRENTE_PONTA;
    } else {
      refEixo =
          perfilEixo.calculate(PERIODO_S, refEixo, new TrapezoidProfile.State(alvoEixo, 0.0));
      eixoPos = refEixo.position;
      eixoVelRef = refEixo.velocity;
    }

    double voltsOmbro =
        pidOmbro.calculate(ombro, refOmbro.position)
            // ArmFeedforward espera o ângulo a partir da horizontal.
            + ffOmbro.calculate(Math.PI / 2.0 + ombro, refOmbro.velocity);
    double voltsEixo =
        pidEixo.calculate(eixo, eixoPos)
            + EIXO_KV * eixoVelRef
            - ArmIOSim.voltsGravidadeNoEixo(ombro, eixo);

    return new Saida(
        MathUtil.clamp(voltsOmbro, -VOLTS_MAX, VOLTS_MAX),
        MathUtil.clamp(voltsEixo, -VOLTS_MAX, VOLTS_MAX));
  }
}
