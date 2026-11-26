package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Números do braço de 2 estágios. Tudo o que está marcado com PREENCHER ainda é
 * provisório.
 *
 * <p>Convenção de ângulos (usada no projeto inteiro): zero = braço esticado para
 * cima. Positivo = inclina para trás. O ângulo da ponta relativo (pontaRel) é
 * medido em relação ao 1º estágio.
 */
public final class ArmConstants {
  private ArmConstants() {}

  // ----- IDs CAN: PREENCHER (provisórios, a simulação não usa) -----
  public static final int CAN_OMBRO_1 = 20; // PREENCHER
  public static final int CAN_OMBRO_2 = 21; // PREENCHER
  public static final int CAN_EIXO_1 = 22; // PREENCHER
  public static final int CAN_EIXO_2 = 23; // PREENCHER

  // ----- Motores: cada grupo tem 2 Kraken X60 no mesmo eixo -----
  public static final DCMotor MOTORES_OMBRO = DCMotor.getKrakenX60(2);
  public static final DCMotor MOTORES_EIXO = DCMotor.getKrakenX60(2);

  // ----- Reduções (do CAD) -----
  private static final double ENGRENAGENS = (60.0 / 16.0) * (60.0 / 16.0);
  /** Grupo A: 16→60, 16→60 e corrente 16→48 = 42,1875:1. */
  public static final double REDUCAO_OMBRO = ENGRENAGENS * (48.0 / 16.0);
  /** Grupo B: 16→60, 16→60 = 14,0625:1 até o eixo do ombro. */
  public static final double REDUCAO_EIXO = ENGRENAGENS;
  /** Corrente do eixo do ombro (16T) até o cotovelo (48T). */
  public static final double CORRENTE_PONTA = 16.0 / 48.0;

  // ----- Comprimentos e massas -----
  public static final double COMPRIMENTO_1 = Units.inchesToMeters(27.74);
  public static final double COMPRIMENTO_2 = Units.inchesToMeters(37.58); // até as rodas da garra
  public static final double MASSA_1_KG = 3.0; // PREENCHER
  public static final double MASSA_2_KG = 2.0; // PREENCHER
  /** Inércia extra no eixo do ombro (eixo, coroas, engrenagens). */
  public static final double INERCIA_EIXO = 0.005; // PREENCHER

  // ----- Posição do ombro em relação ao centro do robô (X frente, Y esquerda) -----
  public static final double OMBRO_X = 0.0; // PREENCHER
  public static final double OMBRO_Y = 0.0; // PREENCHER
  public static final double OMBRO_Z = Units.inchesToMeters(15.7); // medido no CAD

  // ----- Limites (radianos) -----
  public static final double OMBRO_MIN = Units.degreesToRadians(-90.0); // PREENCHER
  public static final double OMBRO_MAX = Units.degreesToRadians(90.0); // PREENCHER
  public static final double PONTA_MIN = Units.degreesToRadians(-150.0); // PREENCHER
  public static final double PONTA_MAX = Units.degreesToRadians(150.0); // PREENCHER
  /** Limite do eixo no teste "só B" (equivale à ponta entre os limites acima). */
  public static final double EIXO_MIN = PONTA_MIN / CORRENTE_PONTA;
  public static final double EIXO_MAX = PONTA_MAX / CORRENTE_PONTA;

  // ----- Posições prontas (graus): ombro e ponta relativa -----
  public enum Posicao {
    /** Pose do CAD: braço recolhido. */
    GUARDADO(5.8, -143.1),
    COLETAR(-30.0, -118.0),
    MEDIO(-10.0, -70.0),
    ALTO(-10.0, -30.0);

    public final double ombroGraus;
    public final double pontaRelGraus;

    Posicao(double ombroGraus, double pontaRelGraus) {
      this.ombroGraus = ombroGraus;
      this.pontaRelGraus = pontaRelGraus;
    }
  }

  // ----- Perfil de movimento (rad/s e rad/s²) -----
  public static final double OMBRO_VEL_MAX = Units.degreesToRadians(120.0);
  public static final double OMBRO_ACEL_MAX = Units.degreesToRadians(240.0);
  public static final double PONTA_VEL_MAX = Units.degreesToRadians(60.0);
  public static final double PONTA_ACEL_MAX = Units.degreesToRadians(120.0);
  public static final double EIXO_VEL_MAX = Units.degreesToRadians(180.0);
  public static final double EIXO_ACEL_MAX = Units.degreesToRadians(360.0);

  // ----- Controle manual pelo analógico (velocidade com o analógico no fim) -----
  public static final double OMBRO_VEL_MANUAL = Units.degreesToRadians(60.0);
  public static final double PONTA_VEL_MANUAL = Units.degreesToRadians(60.0);

  // ----- Ganhos (ajustados só para a simulação) -----
  public static final double OMBRO_KP = 24.0; // volts por radiano
  public static final double OMBRO_KD = 0.0;
  public static final double EIXO_KP = 6.0;
  public static final double EIXO_KD = 0.0;

  public static final double PERIODO_S = 0.02;
  public static final double VOLTS_MAX = 12.0;
  public static final double GRAVIDADE = 9.80665;

  // ----- Valores calculados (física aproximada, braço considerado esticado) -----

  /** Inércia total vista pelo ombro, tubos como barras uniformes. */
  public static final double INERCIA_OMBRO =
      MASSA_1_KG * COMPRIMENTO_1 * COMPRIMENTO_1 / 3.0
          + MASSA_2_KG
              * (COMPRIMENTO_1 * COMPRIMENTO_1
                  + COMPRIMENTO_1 * COMPRIMENTO_2
                  + COMPRIMENTO_2 * COMPRIMENTO_2 / 3.0);

  /** Massa vezes distância do centro de massa até o ombro (kg·m). */
  public static final double MOMENTO_OMBRO =
      MASSA_1_KG * COMPRIMENTO_1 / 2.0 + MASSA_2_KG * (COMPRIMENTO_1 + COMPRIMENTO_2 / 2.0);

  /** Torque de gravidade da ponta em torno do cotovelo, com ela na horizontal (N·m). */
  public static final double TORQUE_PONTA_HORIZONTAL =
      MASSA_2_KG * GRAVIDADE * COMPRIMENTO_2 / 2.0;
}
