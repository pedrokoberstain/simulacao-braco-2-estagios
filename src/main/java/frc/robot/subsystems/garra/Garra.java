package frc.robot.subsystems.garra;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Garra de rodas: 1 motor. Positivo coleta, negativo arremessa. */
public class Garra extends SubsystemBase {
  private double potencia = 0.0;

  public double getPotencia() {
    return DriverStation.isDisabled() ? 0.0 : potencia;
  }

  /** Roda enquanto o botão estiver pressionado. */
  public Command rodar(double novaPotencia) {
    return startEnd(() -> potencia = novaPotencia, () -> potencia = 0.0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Garra/Potencia", getPotencia());
  }
}
