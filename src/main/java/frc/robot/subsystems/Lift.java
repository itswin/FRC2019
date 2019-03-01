/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Lift.LiftCommand;

public class Lift extends PIDSubsystem {
  private CANSparkMax leftLiftMotor;
  private CANSparkMax rightLiftMotor;

  private CANEncoder leftLiftEncoder;
  private CANEncoder rightLiftEncoder;

  private CANDigitalInput leftLiftLimitSwitch;
  private CANDigitalInput rightLiftLimitSwitch;

  // Max and min points the lift is allowed to go to
  private final double minLeftLiftEncoderValue = -100; // -100
  private final double maxLeftLiftEncoderValue = 1; // 1
  private final double minRightLiftEncoderValue = -1; // -1
  private final double maxRightLiftEncoderValue = 100; // 100
  private double leftLiftEncoderComparison = 0;
  private double rightLiftEncoderComparison = 0;

  // Setpoints for the lift
  public static final double kHome = 0;
  public static final double kFirstRocketCargoHole = 28;
  public static final double kSecondRocketCargoHole = 67;
  public static final double kThirdRocketCargoHole = 99;
  public static final double kFirstRocketHatch = 0;
  public static final double kSecondRocketHatch = 45;
  public static final double kThirdRocketHatch = 80;
  public static final double kStationHatch = 9;

  // PID constants
  private final double kAbsoluteTolerance = 1;
  private final double kPercentTolerance = 3;

  public Lift() {
    super("Lift", .1, 0, 0, 0, .01);

    leftLiftMotor = new CANSparkMax(RobotMap.leftLiftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLiftMotor = new CANSparkMax(RobotMap.rightLiftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftLiftEncoder = new CANEncoder(leftLiftMotor);
    rightLiftEncoder = new CANEncoder(rightLiftMotor);

    leftLiftLimitSwitch = leftLiftMotor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
    rightLiftLimitSwitch = rightLiftMotor.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

    leftLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftLiftMotor.setInverted(true);
    rightLiftMotor.setInverted(false);

    // Setup the PID
    setOutputRange(-1, 1);
    setInputRange(minRightLiftEncoderValue - 5, maxRightLiftEncoderValue + 5);
    setAbsoluteTolerance(kAbsoluteTolerance);
    setPercentTolerance(kPercentTolerance);
    enable();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LiftCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return getEncoderAverage();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    // System.out.println("PID: " + output);
    if(output < 0) {
      output *= .5;
    }
    setPower(output);
  }

  // ********** Motor methods ********** //
  public void setPower(double speed) {
    leftLiftMotor.set(speed);
    rightLiftMotor.set(speed);
  }

  public void stopLift() {
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);
  }
  
  // ********** Encoder methods ********** //
  public double getLeftLiftEncoder() {
    return leftLiftEncoder.getPosition() - leftLiftEncoderComparison;
  }

  public double getRightLiftEncoder() {
    return rightLiftEncoder.getPosition() - rightLiftEncoderComparison;
  }

  public double getTrueLeftLiftEncoder() {
    return leftLiftEncoder.getPosition();
  }

  public double getTrueRightLiftEncoder() {
    return rightLiftEncoder.getPosition();
  }

  public double getEncoderAverage() {
    return (getRightLiftEncoder() - getLeftLiftEncoder()) / 2;
  }

  // Used to programmatically stop lift at encoder values
  // These were necessary before limit switches were installed
  public boolean canLeftLiftAscend() {
    return getLeftLiftEncoder() < maxLeftLiftEncoderValue;
  }

  public boolean canLeftLiftDescend() {
    return getLeftLiftEncoder() > minLeftLiftEncoderValue;
  }

  public boolean canRightLiftAscend() {
    return getRightLiftEncoder() > minRightLiftEncoderValue;
  }

  public boolean canRightLiftDescend() {
    return getRightLiftEncoder() < maxRightLiftEncoderValue;
  }

  public double getMaxRightLiftEncoderValue() {
    return maxRightLiftEncoderValue;
  }

  public boolean getLeftLimitSwitchVal() {
    return leftLiftLimitSwitch.get();
  }

  public boolean getRightLimitSwitchVal() {
    return rightLiftLimitSwitch.get();
  }

  public void setEncoderComparisons(double leftComparison, double rightComparison) {
    leftLiftEncoderComparison = leftComparison;
    rightLiftEncoderComparison = rightComparison;
  }
}
