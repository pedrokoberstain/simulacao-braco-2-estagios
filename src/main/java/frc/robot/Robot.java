package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  @SuppressWarnings("unused")
  private final RobotContainer container;

  public Robot() {
    Logger.addDataReceiver(new WPILOGWriter(isReal() ? "/home/lvuser/logs" : "logs"));
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
