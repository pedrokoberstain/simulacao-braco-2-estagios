package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.robot.subsystems.arm.ArmConstants.*;

/**
 * Simulação. O ombro usa SingleJointedArmSim. O eixo do ombro é um motor com
 * inércia, carregado com o peso da ponta dividido pela corrente (1/3).
 *
 * <p>Aproximações: a inércia do braço é calculada com ele esticado, e a reação
 * da corrente sobre o ombro é ignorada. O acoplamento de ângulos é exato (ver
 * ArmKinematics).
 */
public class ArmIOSim implements ArmIO {
  // SingleJointedArmSim trata o braço como barra uniforme. Este comprimento
  // equivalente faz a gravidade dele bater com a do nosso braço de 2 massas.
  private static final double COMPRIMENTO_EQUIVALENTE =
      1.5 * INERCIA_OMBRO / MOMENTO_OMBRO;

  // O WPILib mede o ângulo do braço a partir da horizontal; nós, da vertical.
  private static final double VERTICAL = Math.PI / 2.0;

  private final SingleJointedArmSim ombro =
      new SingleJointedArmSim(
          MOTORES_OMBRO,
          REDUCAO_OMBRO,
          INERCIA_OMBRO,
          COMPRIMENTO_EQUIVALENTE,
          VERTICAL + OMBRO_MIN,
          VERTICAL + OMBRO_MAX,
          true,
          VERTICAL);

  private static final double INERCIA_NO_EIXO =
      MASSA_2_KG * COMPRIMENTO_2 * COMPRIMENTO_2 / 3.0 * CORRENTE_PONTA * CORRENTE_PONTA
          + INERCIA_EIXO;

  private final DCMotorSim eixo =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(MOTORES_EIXO, INERCIA_NO_EIXO, REDUCAO_EIXO),
          MOTORES_EIXO);

  private double ombroVolts = 0.0;
  private double eixoVolts = 0.0;
  private boolean freio = false;

  /** Começa com o braço esticado para cima. */
  public ArmIOSim() {}

  /** Começa na posição dada (radianos). */
  public ArmIOSim(double ombroRad, double pontaRelRad) {
    ombro.setState(VERTICAL + ombroRad, 0.0);
    eixo.setState(ArmKinematics.eixoPara(ombroRad, pontaRelRad), 0.0);
  }

  @Override
  public void setFreio(boolean freio) {
    this.freio = freio;
  }

  /** Volts que equivalem ao torque que o peso da ponta faz no eixo. */
  static double voltsGravidadeNoEixo(double ombroRad, double eixoRad) {
    double absoluta =
        ArmKinematics.pontaAbsoluta(ombroRad, ArmKinematics.pontaRel(eixoRad, ombroRad));
    // Positivo = ponta inclinada para trás, o peso puxa ainda mais para trás.
    double torqueNoCotovelo = TORQUE_PONTA_HORIZONTAL * Math.sin(absoluta);
    double torqueNoEixo = torqueNoCotovelo * CORRENTE_PONTA;
    return torqueNoEixo * MOTORES_EIXO.rOhms / (REDUCAO_EIXO * MOTORES_EIXO.KtNMPerAmp);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    double q = ombro.getAngleRads() - VERTICAL;
    double s = eixo.getAngularPositionRad();

    ombro.setInputVoltage(ombroVolts);
    eixo.setInputVoltage(eixoVolts + voltsGravidadeNoEixo(q, s));
    if (freio) {
      ombro.setState(VERTICAL + q, 0.0);
      eixo.setState(s, 0.0);
    } else {
      ombro.update(PERIODO_S);
      eixo.update(PERIODO_S);
    }

    inputs.ombroPosicaoRad = ombro.getAngleRads() - VERTICAL;
    inputs.ombroVelocidadeRadS = ombro.getVelocityRadPerSec();
    inputs.ombroVolts = ombroVolts;
    inputs.ombroCorrenteA = Math.abs(ombro.getCurrentDrawAmps());
    inputs.eixoPosicaoRad = eixo.getAngularPositionRad();
    inputs.eixoVelocidadeRadS = eixo.getAngularVelocityRadPerSec();
    inputs.eixoVolts = eixoVolts;
    inputs.eixoCorrenteA =
        Math.abs(
            MOTORES_EIXO.getCurrent(
                eixo.getAngularVelocityRadPerSec() * REDUCAO_EIXO, eixoVolts));
  }

  @Override
  public void setOmbroVolts(double volts) {
    ombroVolts = MathUtil.clamp(volts, -VOLTS_MAX, VOLTS_MAX);
  }

  @Override
  public void setEixoVolts(double volts) {
    eixoVolts = MathUtil.clamp(volts, -VOLTS_MAX, VOLTS_MAX);
  }
}
