package frc.team7587.robot;

//essentials?
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

//subsystems
import frc.team7587.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {

  public static OI m_oi = new OI();
  public static DriveTrain m_driveTrain = new DriveTrain();

  Command m_autoCommand;
  // SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    
    // m_autoCommand = new AutoCmdGroup();

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // schedule the autonomous command
    if (m_autoCommand != null) {
      m_autoCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    log();
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  public void log() {
    // m_driveTrain.log();
  }
}
