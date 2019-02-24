/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.PIDDriveTrain.PIDDriveTrainCommand;

/**
 * Add your docs here.
 */
public class PIDDriveTrain extends PIDSubsystem {
  
  // Motors
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private final int kCurrentLimit = 35;

  private CANSparkMax.IdleMode currentIdleMode;

  private MecanumDrive robotDrive;

  // Encoders
  private CANEncoder frontLeftEnc;
  private CANEncoder backLeftEnc;
  private CANEncoder backRightEnc;
  private CANEncoder frontRightEnc;

	public final double kWheel_diameter = 0.2032; // Meters
  public final double kPulsesPerRevolution = 360; // TODO: CHANGE?
  public final double kDistanceScaler = 16; // Gear ratio plus tuning TODO: Tune

  private final double kAbsoluteTolerance = 2;
  private final double kPercentTolerance = 1;

  private double rotationSpeed = 0;

  /**
   * Add your docs here.
   */
  public PIDDriveTrain() {
    // Intert a subsystem name and PID values here
    super("PIDDriveTrain", .025, 0, 0, 0, .01);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    frontLeft = new CANSparkMax(RobotMap.frontLeftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeft = new CANSparkMax(RobotMap.backLeftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontRight = new CANSparkMax(RobotMap.frontRightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    backRight = new CANSparkMax(RobotMap.backRightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    frontLeft.setSmartCurrentLimit(kCurrentLimit);
    frontRight.setSmartCurrentLimit(kCurrentLimit);
    backLeft.setSmartCurrentLimit(kCurrentLimit);
    backRight.setSmartCurrentLimit(kCurrentLimit);

    changeIdleMode(CANSparkMax.IdleMode.kCoast);
    
    frontLeftEnc = new CANEncoder(frontLeft);
    backLeftEnc = new CANEncoder(backLeft);
    backRightEnc = new CANEncoder(backRight);
    frontRightEnc = new CANEncoder(frontRight);

    robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  
    setOutputRange(-.25, .25);
    setInputRange(-360, 360);
    setAbsoluteTolerance(kAbsoluteTolerance);
    setPercentTolerance(kPercentTolerance);
    enable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new PIDDriveTrainCommand());
  }

  public void drive(double speed, double strafe, double rotation) {
    robotDrive.driveCartesian(strafe, speed, rotation);
  }

  public void drive(double speed, double strafe, double rotation, double angle) {
    robotDrive.driveCartesian(strafe, speed, rotation, angle);
  }

  public void stopDriveTrain() {
    robotDrive.driveCartesian(0, 0, 0);
  }

  public void resetEncoders() {
  }

  public double getFrontLeftEncoderPosition() {
    return frontLeftEnc.getPosition();
  }

  public double getBackLeftEncoderPosition() {
    return backLeftEnc.getPosition();
  }

  public double getFrontRightEncoderPosition() {
    return frontRightEnc.getPosition();
  }

  public double getbackRightEncoderPosition() {
    return backRightEnc.getPosition();
  }

  public double getLeftEncoderAverage() {
    return (frontLeftEnc.getPosition() + backLeftEnc.getPosition()) / 2;
  }

  public double getRightEncoderAverage() {
    return (frontRightEnc.getPosition() + backRightEnc.getPosition()) / 2;
  }

  public CANSparkMax.IdleMode getIdleMode() {
    return currentIdleMode;
  }

  public void setLeftSpeed(double speed) {
    backLeft.set(speed);
    frontLeft.set(speed);
  }

  public void setRightSpeed(double speed) {
    backRight.set(speed);
    frontRight.set(speed);
  }

  public void changeIdleMode(CANSparkMax.IdleMode idleMode) {
    frontLeft.setIdleMode(idleMode);
    backLeft.setIdleMode(idleMode);
    frontRight.setIdleMode(idleMode);
    backRight.setIdleMode(idleMode);

    currentIdleMode = idleMode;
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return Robot.m_navX.getAngle() % 360;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    rotationSpeed = output;
    // System.out.println("Output: " + output);
  }

  public double getRotationSpeed() {
    return rotationSpeed;
  }

  public void setSetpoint360(double setpoint) {
    setSetpoint(setpoint % 360);
  }
}
