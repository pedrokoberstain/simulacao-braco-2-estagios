package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;

/** Robô inteiro na simulação: o analógico anda e os botões não quebram nada. */
class RobotContainerTest {
  private XboxControllerSim xbox;

  private void ciclos(int n) {
    for (int i = 0; i < n; i++) {
      xbox.notifyNewData();
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  void analogicoParaFrenteAndaEBotoesFuncionam() {
    HAL.initialize(500, 0);
    xbox = new XboxControllerSim(0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    xbox.setAxisCount(6);
    xbox.setButtonCount(10);
    RobotContainer robo = new RobotContainer();
    ciclos(5);
    double xInicial = robo.poseRobo().getX();

    xbox.setRightTriggerAxis(1.0);
    ciclos(100);
    assertTrue(robo.poseRobo().getX() > xInicial + 1.0, "andou " + robo.poseRobo());
    assertEquals(2.0, robo.poseRobo().getY(), 0.05);

    xbox.setRightTriggerAxis(0.0);
    ciclos(100);
    double xParado = robo.poseRobo().getX();
    xbox.setLeftTriggerAxis(1.0); // LT: ré
    ciclos(100);
    assertTrue(robo.poseRobo().getX() < xParado - 1.0, "ré " + robo.poseRobo());
    xbox.setLeftTriggerAxis(0.0);

    // Analógico direito para cima: só o ombro se move.
    double ombro0 = robo.ombroGraus();
    double ponta0 = robo.pontaRelGraus();
    xbox.setRightY(-1.0);
    ciclos(50);
    xbox.setRightY(0.0);
    ciclos(50);
    assertTrue(robo.ombroGraus() < ombro0 - 30.0, "ombro " + robo.ombroGraus());
    assertEquals(ponta0, robo.pontaRelGraus(), 1.0);

    // Analógico direito para o lado: só o cotovelo se move.
    double ombro1 = robo.ombroGraus();
    xbox.setRightX(1.0);
    ciclos(50);
    xbox.setRightX(0.0);
    ciclos(50);
    assertTrue(robo.pontaRelGraus() > ponta0 + 30.0, "ponta " + robo.pontaRelGraus());
    assertEquals(ombro1, robo.ombroGraus(), 1.0);

    // Solto: continua parado.
    double ponta2 = robo.pontaRelGraus();
    ciclos(100);
    assertEquals(ponta2, robo.pontaRelGraus(), 0.5);

    xbox.setBButton(true);
    xbox.setLeftBumperButton(true);
    ciclos(200);
    assertEquals(-30.0, robo.ombroGraus(), 0.5);

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }
}
