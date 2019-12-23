package frc.team7587.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team7587.robot.OI;
import frc.team7587.robot.Utl;
import frc.team7587.robot.commands.TeleopDrive;

public class DriveTrain extends Subsystem implements PIDOutput{

  private final SpeedController m_leftMotor = new PWMVictorSPX(OI.LEFT_MOTOR);
  private final SpeedController m_rightMotor = new PWMVictorSPX(OI.RIGHT_MOTOR);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

  private ADXRS450_Gyro m_gyro;

  static final double kP = 0.05;
  static final double kI = 0.000;
  static final double kD = 0.02;
  static final double kF = 0.00;

  public static final double kToleranceDegrees = 3.0f;

  PIDController turnController;
  double rotateToAngleRate;

  public DriveTrain() {
    
    Utl.log0("===DriveTrain constructor");

    m_gyro = new ADXRS450_Gyro(kGyroPort);    
    m_gyro.calibrate();
    m_gyro.reset();
    // PIDSourceType.kDisplacement is default
    // m_gyro.setPIDSourceType(PIDSourceType.kDisplacement);

    turnController = new PIDController(kP, kI, kD, kF, m_gyro, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-0.65, 0.65);
    
    turnController.setAbsoluteTolerance(kToleranceDegrees);
    turnController.setContinuous(true);
  
    showGyroData();
  }

  @Override
  public void initDefaultCommand() {
    Utl.log0("driveTrain.initDefaultCommand(): teleOpDrive()");
    setDefaultCommand(new TeleopDrive());
  }

  public void showGyroData(){
    SmartDashboard.putString("gyro name", m_gyro.getName());
    SmartDashboard.putNumber("gyro rate", m_gyro.getRate());
    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("PID output", this.rotateToAngleRate);
    SmartDashboard.putNumber("PID error", turnController.getError());
  }


  public PIDController getTurnController(){
    return this.turnController;
  }
  
  public void resetGyro(){
    Utl.log0("...driveTrain.resetGyro()");
    m_gyro.reset();
    turnController.reset();
    Timer.delay(0.1);
  }

  public void initGyro(double targetAngle){
    Utl.log0("...driveTrain.initGyro()");
    // turnController.reset();
    // m_gyro.reset();
    turnController.setSetpoint(targetAngle);
    turnController.enable();
  }

  public void rotateAngle(){
    // use the rotateAngleRate from PID controller to turn
    showGyroData();
    Utl.log(" ## driveTrain.rotateAngle(), PID output: " + this.rotateToAngleRate);
    m_drive.arcadeDrive(0, this.rotateToAngleRate);
  }

  public double getGyroAngle(){
    return m_gyro.getAngle();
  }

  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }

  /* normal teleOp drive by joystick */
  public void drive(double speed, double rotation) {
    // Utl.log0("teleOp rotation: " + rotation);
    SmartDashboard.putNumber("TeleOp drive rotation: ", rotation);
    m_drive.arcadeDrive(speed, rotation);
  }

  public void stop(){
    m_drive.arcadeDrive(0,0);
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    
  }

}
