/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.*;

import frc.robot.RobotMap;
import frc.robot.commands.DriveTrain.*;

import jaci.pathfinder.*;

public class DriveTrain extends Subsystem {
  // Motors
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

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

  public DriveTrain() {
    frontLeft = new CANSparkMax(RobotMap.frontLeftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeft = new CANSparkMax(RobotMap.backLeftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontRight = new CANSparkMax(RobotMap.frontRightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    backRight = new CANSparkMax(RobotMap.backRightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    changeIdleMode(CANSparkMax.IdleMode.kBrake);
    
    frontLeftEnc = new CANEncoder(frontLeft);
    backLeftEnc = new CANEncoder(backLeft);
    backRightEnc = new CANEncoder(backRight);
    frontRightEnc = new CANEncoder(frontRight);

    robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveTrainCommand());
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

  public void setSlaveMode(boolean isSlaveMode) {
    // TODO? Can we unset slave mode?
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
}