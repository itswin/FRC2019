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

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.PIDs.*;
import frc.robot.commands.DriveTrain.DriveTrainBaseCommand;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Motors
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private final int kCurrentLimit = 35;

  private CANSparkMax.IdleMode currentIdleMode;

  private MecanumDrive m_drive;
  
  private CANEncoder frontLeftEnc;
  private CANEncoder backLeftEnc;
  private CANEncoder backRightEnc;
  private CANEncoder frontRightEnc;

  // Hatches and cargo are placed at fixed angles
  public static final double[] kScoringAngles = {-151.25, -90, -28.75, 0, 28.75, 90, 151.25, 180};
  public double zeroAngle = 0;

  // PIDs
  private final double kRotationP = .05;
  private final double kRotationI = 0.0001;
  private final double kRotationD = 0.2;
  private final double kRotationF = 0;
  private final double kRotationPeriod = .01;
  private final double kRotationAbsoluteTolerance = 2;
  public static final double rotationThreshold = .5;
  public PIDController rotationPIDController;
  public DriveTrainRotationPIDOutput rotationPIDOutput;

  public final double kHorizontalBaseline = .15;
  // private double kHorizontalP = .15;
  // private double kHorizontalI = 0.00075; //.001
  // private double kHorizontalD = 1.5; // .5
  // private double kHorizontalF = 0;
  // private double kHorizontalMax = .4;
  // private double kHorizontalMin = -.4;
  private double kHorizontalP = .1;
  private double kHorizontalI = 0.00075;
  private double kHorizontalD = 1;
  private double kHorizontalF = 0;
  private double kHorizontalMax = .3;
  private double kHorizontalMin = -.3;
  private final double kHorizontalPeriod = 0.01;
  private final double kHorizontalAbsoluteTolerance = .5;
  public static final double kHorizontalSetpoint = -.9; // -.43
  public PIDController horizontalPIDController;
  public HorizontalDistancePIDSource horizontalDistancePIDSource;
  public DriveTrainStrafePIDOutput strafePIDOutput;

  private final double kVerticalP = 0.01;
  private final double kVerticalI = 0.001;
  private final double kVerticalD = .5;
  private final double kVerticalF = 0;
  private final double kVerticalPeriod = 0.01;
  private final double kVerticalAbsoluteTolerance = 2;
  public final static double kVerticalTarget = 11.25;
  public PIDController verticalPIDController;
  public VerticalDistancePIDSource verticalDistancePIDSource;
  public DriveTrainSpeedPIDOutput speedPIDOutput;
  
  // Joystick speeds updated in periodic
  private double inputJoystickSpeed = 0;
  private double inputJoystickStrafeSpeed = 0;
  private double inputJoystickRotationSpeed = 0;

  // Auto speeds
  private double inputAutoSpeed = 0;
  private double inputAutoStrafeSpeed = 0;
  private double inputAutoRotationSpeed = 0;

  // Current speed applied to motors
  private double currentSpeed = 0;
  private double currentStrafeSpeed = 0;
  private double currentRotationSpeed = 0;

  public DriveState driveState = DriveState.kManual;
  public enum DriveState {
    kAuto, kManual, kAutoHorizontal
  }

  public DriveTrain() {
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

    m_drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveTrainBaseCommand());
  }

  // Needs to be done after DriveTrain is initialized
  public void initPIDs() {
    rotationPIDOutput = new DriveTrainRotationPIDOutput();
    rotationPIDController = new PIDController(kRotationP, kRotationI, kRotationD, kRotationF, Robot.m_navX, rotationPIDOutput, kRotationPeriod);
    rotationPIDController.setOutputRange(-.25, .25);
    rotationPIDController.setInputRange(-180, 180);
    rotationPIDController.setAbsoluteTolerance(kRotationAbsoluteTolerance);
    rotationPIDController.setContinuous();
    rotationPIDController.disable();

    horizontalDistancePIDSource = new HorizontalDistancePIDSource();
    strafePIDOutput = new DriveTrainStrafePIDOutput();
    horizontalPIDController = new PIDController(kHorizontalP, kHorizontalI, kHorizontalD, kHorizontalF, horizontalDistancePIDSource, strafePIDOutput, kHorizontalPeriod);
    horizontalPIDController.setOutputRange(kHorizontalMin, kHorizontalMax);
    horizontalPIDController.setAbsoluteTolerance(kHorizontalAbsoluteTolerance);
    horizontalPIDController.setSetpoint(0);
    horizontalPIDController.disable();

    verticalDistancePIDSource = new VerticalDistancePIDSource();
    speedPIDOutput = new DriveTrainSpeedPIDOutput();
    verticalPIDController = new PIDController(kVerticalP, kVerticalI, kVerticalD, kVerticalF, verticalDistancePIDSource, speedPIDOutput, kVerticalPeriod);
    verticalPIDController.setOutputRange(-.25, .25);
    verticalPIDController.setAbsoluteTolerance(kVerticalAbsoluteTolerance);
    verticalPIDController.setSetpoint(0);
    verticalPIDController.disable();
  }

  public void drive(double speed, double strafe, double rotation) {
    m_drive.driveCartesian(strafe, speed, rotation);
  }
  
  public void drive(double speed, double strafe, double rotation, double angle) {
    m_drive.driveCartesian(strafe, speed, rotation, angle);
  }

  public void stop() {
    rotationPIDController.setSetpoint(Robot.getComparedYaw());

    currentSpeed = 0;
    currentStrafeSpeed = 0;
    currentRotationSpeed = 0;
    drive(0, 0, 0);
  }

  public void changeIdleMode(CANSparkMax.IdleMode idleMode) {
    frontLeft.setIdleMode(idleMode);
    backLeft.setIdleMode(idleMode);
    frontRight.setIdleMode(idleMode);
    backRight.setIdleMode(idleMode);

    currentIdleMode = idleMode;
  }
  
  // ********** Methods so that updating inputs work from periodic ********** //
  public void setInputJoystickSpeeds(double speed, double strafe, double rotation) {
    inputJoystickSpeed = speed;
    inputJoystickStrafeSpeed = strafe;
    inputJoystickRotationSpeed = rotation;
  }

  public double getInputJoystickSpeed() {
    return inputJoystickSpeed;
  }

  public double getInputJoystickStrafeSpeed() {
    return inputJoystickStrafeSpeed;
  }

  public double getInputJoystickRotationSpeed() {
    return inputJoystickRotationSpeed;
  }
  
  // ********** Auto Input speed methods ********** //
  public void setInputAutoSpeeds(double speed, double strafe, double rotation) {
    inputAutoSpeed = speed;
    inputAutoStrafeSpeed = strafe;
    inputAutoRotationSpeed = rotation;
  }

  public double getInputAutoSpeed() {
    return inputAutoSpeed;
  }

  public double getInputAutoStrafeSpeed() {
    return inputAutoStrafeSpeed;
  }

  public double getInputAutoRotationSpeed() {
    return inputAutoRotationSpeed;
  }

  public void setInputAutoSpeed(double speed) {
    inputAutoSpeed = speed;
  }

  public void setInputAutoStrafeSpeed(double strafe) {
    inputAutoStrafeSpeed = strafe;
  }

  public void setInputAutoRotationSpeed(double rotation) {
    inputAutoRotationSpeed = rotation;
  }

  // ********** Current speed methods ********** //
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

  public double getCurrentSpeed() {
    return currentSpeed;
  }

  public double getCurrentStrafeSpeed() {
    return currentStrafeSpeed;
  }

  public double getCurrentRotationSpeed() {
    return currentRotationSpeed;
  }

  public void enableCenteringPIDs() {
    verticalPIDController.enable();
    horizontalPIDController.enable();
  }

  public void disableCenteringPIDs() {
    verticalPIDController.disable();
    horizontalPIDController.disable();
  }

  public boolean centeringPIDsOnTarget() {
    return verticalPIDController.onTarget() && horizontalPIDController.onTarget();
  }
  
  public double getClosestScoringAngle() {
    double currentAngle = Robot.m_navX.getYaw();

    int index = 0;
    double difference = 1000; // Big starting number guarantees first loop will store a new difference
    for(int i = 0; i < DriveTrain.kScoringAngles.length; i++) {
      if(Math.abs(DriveTrain.kScoringAngles[i] - currentAngle) < difference) {
        index = i;
        difference = Math.abs(DriveTrain.kScoringAngles[i] - currentAngle);
      }
    }

    // System.out.println(DriveTrain.kScoringAngles[index]);
    return DriveTrain.kScoringAngles[index];
  }
}
