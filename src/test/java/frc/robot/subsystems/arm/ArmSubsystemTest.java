package frc.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

/** Subsistema completo: robô habilitado, comando agendado e ciclos de 20 ms. */
class ArmSubsystemTest {
  @Test
  void comandoCompensadoMoveOBracoComORoboHabilitado() {
    HAL.initialize(500, 0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    ArmIOSim sim = new ArmIOSim();
    ArmIO.ArmIOInputs ultimo = new ArmIO.ArmIOInputs();
    Arm braco =
        new Arm(
            new ArmIO() {
              @Override
              public void updateInputs(ArmIOInputs inputs) {
                sim.updateInputs(inputs);
                ultimo.ombroPosicaoRad = inputs.ombroPosicaoRad;
                ultimo.eixoPosicaoRad = inputs.eixoPosicaoRad;
              }

              @Override
              public void setOmbroVolts(double volts) {
                sim.setOmbroVolts(volts);
              }

              @Override
              public void setEixoVolts(double volts) {
                sim.setEixoVolts(volts);
              }
            });

    CommandScheduler.getInstance().schedule(braco.compensado(() -> 30.0, () -> 20.0));
    for (int i = 0; i < 200; i++) {
      CommandScheduler.getInstance().run();
    }

    assertEquals(30.0, Units.radiansToDegrees(ultimo.ombroPosicaoRad), 0.5);
    assertEquals(
        20.0,
        Units.radiansToDegrees(
            ArmKinematics.pontaRel(ultimo.eixoPosicaoRad, ultimo.ombroPosicaoRad)),
        0.5);

    CommandScheduler.getInstance().unregisterSubsystem(braco);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }
}
