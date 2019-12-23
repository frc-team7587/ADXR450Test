/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7587.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team7587.robot.Robot;
import frc.team7587.robot.Utl;
import frc.team7587.robot.subsystems.DriveTrain;

public class AutoRotate extends Command {

  private double rotateAngle;
  private int inErrZoneCount;

  public AutoRotate(double rotateAngle) {
    // requires(Robot.m_driveTrain);
    Utl.log0("....autoRotate cmd constructor");
    this.rotateAngle = rotateAngle;

    Robot.m_driveTrain.resetGyro();
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Utl.log0("....autoRotate cmd init");
    Robot.m_driveTrain.resetGyro();
    Robot.m_driveTrain.initGyro(rotateAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_driveTrain.rotateAngle();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    // this may not work per some posts
    // double err = Robot.m_driveTrain.getTurnController().getError();
    // boolean ctrlOnTarget = Robot.m_driveTrain.getTurnController().onTarget();
    // Utl.log0("gyro on target: " + ctrlOnTarget);
    // return ctrlOnTarget;

    double tolerance = DriveTrain.kToleranceDegrees;
    double err = Math.abs(Robot.m_driveTrain.getGyroAngle() - this.rotateAngle);
    if( Math.abs(err) < tolerance ){
      boolean done = (++inErrZoneCount) >= 5;
      Utl.log0(" ### autoRotate.isFinished() diff=" + err + "; inZone count=" + inErrZoneCount + "done? " + done);
      return done;   // after 5 consecutive inZone test
    }else{
      inErrZoneCount = 0;
    }
    return false;
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Utl.log(" ## autoRotate.end()");
    Robot.m_driveTrain.stop();
    Robot.m_driveTrain.getTurnController().disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}
