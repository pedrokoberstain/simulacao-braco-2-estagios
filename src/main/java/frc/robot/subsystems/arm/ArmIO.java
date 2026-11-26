package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/**
 * Camada de hardware do braço (padrão AdvantageKit). Hoje só existe a versão de
 * simulação; a versão real (Kraken X60) entra depois, sem mudar o resto.
 */
public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    /** Ângulo do 1º estágio em relação à vertical (rad). */
    public double ombroPosicaoRad = 0.0;

    public double ombroVelocidadeRadS = 0.0;
    public double ombroVolts = 0.0;
    public double ombroCorrenteA = 0.0;

    /** Ângulo do eixo do ombro em relação à torre (rad). */
    public double eixoPosicaoRad = 0.0;

    public double eixoVelocidadeRadS = 0.0;
    public double eixoVolts = 0.0;
    public double eixoCorrenteA = 0.0;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Trava as duas articulações (robô desabilitado). */
  default void setFreio(boolean freio) {}

  /** Grupo A (ombro). */
  default void setOmbroVolts(double volts) {}

  /** Grupo B (eixo do ombro, que move a ponta). */
  default void setEixoVolts(double volts) {}
}
