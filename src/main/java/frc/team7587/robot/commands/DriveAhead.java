package frc.team7587.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team7587.robot.Robot;
import frc.team7587.robot.Utl;
import frc.team7587.robot.subsystems.DriveTrain;

public class DriveAhead extends TimedCommand{

    public DriveAhead(double secs){
        super(secs);
        requires(Robot.m_driveTrain);
        
    }

   @Override
    protected void execute() {

        Robot.m_driveTrain.drive(0.5,0);
        
    }

    // @Override
    // protected boolean isFinished() {
    //     // return true;
    //     return super.isFinished();
    // }

    @Override
    protected void end() {
        Robot.m_driveTrain.drive(0,0);
        Timer.delay(0.5);
        Robot.m_driveTrain.stop();
    }

}