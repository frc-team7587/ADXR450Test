package frc.team7587.robot.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.team7587.robot.Robot;
import frc.team7587.robot.Utl;

/**
 * Have the robot drive tank style using the logitech F310 gamepad
 */
public class TeleopDrive extends Command {

  Joystick stick = Robot.m_oi.getLogiJoy();

  public TeleopDrive() {
    Utl.log("TeleopDrive() constructor");

    requires(Robot.m_driveTrain);
  }

  @Override
  protected void execute() {
    // Utl.log("TeleopDrive() executed", 100);

    Robot.m_driveTrain.drive(stick.getThrottle() * stick.getY(),
                     0.75 * stick.getTwist() * Math.abs(stick.getThrottle()));
  }

  @Override
  protected boolean isFinished() {
    return false; // Runs until interrupted
  }

  @Override
  protected void end() {

  }

}