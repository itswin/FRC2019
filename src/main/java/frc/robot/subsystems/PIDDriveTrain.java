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
 * Create drivetrain as PID to fix strafing using the navX gyro
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

  private CANEncoder frontLeftEnc;
  private CANEncoder backLeftEnc;
  private CANEncoder backRightEnc;
  private CANEncoder frontRightEnc;

  // For use with Pathfinder
	public final double kWheel_diameter = 0.2032; // Meters
  public final double kPulsesPerRevolution = 360; // TODO: CHANGE?
  public final double kDistanceScaler = 16; // Gear ratio plus tuning TODO: Tune

  // PID constants
  private final double kAbsoluteTolerance = 2;

  // Joystick speeds updated in periodic
  private double inputSpeed = 0;
  private double inputStrafeSpeed = 0;
  private double inputRotationSpeed = 0;

  private boolean manualDriveOn = true;

  // Current speed applied to motors
  private double currentSpeed = 0;
  private double currentStrafeSpeed = 0;
  private double currentRotationSpeed = 0;

  // Rotation speed given by the PID to fix strafing
  private double pidRotationSpeed = 0;

  public PIDDriveTrain() {
    super("PIDDriveTrain", .025, 0, 0, 0, .01);

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
  
    // Setup the PID
    setOutputRange(-.25, .25);
    setInputRange(-360, 360);
    setAbsoluteTolerance(kAbsoluteTolerance);
    enable();
  }

  @Override
  public void initDefaultCommand() {
    // Default command sets power to motors from input speeds
    setDefaultCommand(new PIDDriveTrainCommand());
  }

  // Robot oriented driving
  public void drive(double speed, double strafe, double rotation) {
    robotDrive.driveCartesian(strafe, speed, rotation);
  }

  // Field oriented driving
  public void drive(double speed, double strafe, double rotation, double angle) {
    robotDrive.driveCartesian(strafe, speed, rotation, angle);
  }

  public void stopDriveTrain() {
    // Reset the angle setpoint for the PID when stopped
    setSetpoint360(Robot.m_navX.getAngle());

    currentSpeed = 0;
    currentStrafeSpeed = 0;
    currentRotationSpeed = 0;
    robotDrive.driveCartesian(0, 0, 0);
  }

  // ********** Pathfinder methods ********** //
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

  // *********** PID methods *********** //
  /**
   * The PID has a set angle that it tries to stay at while driving
   * and uses the navX current angle as an input.
   */
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
    pidRotationSpeed = output;
  }

  public double getPidRotationSpeed() {
    return pidRotationSpeed;
  }

  public void setSetpoint360(double setpoint) {
    setSetpoint(setpoint % 360);
  }

  // ********** Methods so that updating inputs work from periodic ********** //
  public void setInputSpeeds(double speed, double strafe, double rotation) {
    inputSpeed = speed;
    inputStrafeSpeed = strafe;
    inputRotationSpeed = rotation;
  }

  public double getInputSpeed() {
    return inputSpeed;
  }

  public double getInputStrafeSpeed() {
    return inputStrafeSpeed;
  }

  public double getInputRotationSpeed() {
    return inputRotationSpeed;
  }

  public double getCurrentSpeed() {
    return currentSpeed;
  }

  public double getCurrentStrafeSpeed() {
    return currentStrafeSpeed;
  }

  public double getCurrentRotationSpeed() {
    return currentRotationSpeed;
  }

  public void setCurrentSpeeds(double speed, double strafe, double rotation) {
    currentSpeed = speed;
    currentStrafeSpeed = strafe;
    currentRotationSpeed = rotation;
  }

  public void setCurrentSpeed(double speed) {
    currentSpeed = speed;
  }

  public void setCurrentStrafeSpeed(double strafe) {
    currentStrafeSpeed = strafe;
  }

  public void setCurrentRotationSpeed(double rotation) {
    currentRotationSpeed = rotation;
  }

  public void setManualDriveOn(boolean bool) {
    manualDriveOn = bool;
  }

  public boolean getManualDriveOn() {
    return manualDriveOn;
  }
}
