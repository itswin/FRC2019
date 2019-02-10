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
import frc.robot.RobotMap;
import frc.robot.commands.Lift.LiftCommand;

/**
 * Add your docs here.
 */
public class Lift extends PIDSubsystem {
  private CANSparkMax leftLiftMotor;
  private CANSparkMax rightLiftMotor;

  private CANEncoder leftLiftEncoder;
  private CANEncoder rightLiftEncoder;

  private final double minLeftLiftEncoderValue = -100; // -100
  private final double maxLeftLiftEncoderValue = 1; // 1
  private final double minRightLiftEncoderValue = -1; // -1
  private final double maxRightLiftEncoderValue = 100; // 100

  public static final double kHome = 0;
  public static final double kFirstRocketCargoHole = 30;
  public static final double kSecondRocketCargoHole = 67;
  public static final double kThirdRocketCargoHole = 99;
  public static final double kFirstRocketHatch = 0;
  public static final double kSecondRocketHatch = 45;
  public static final double kThirdRocketHatch = 80;

  private final double kAbsoluteTolerance = 1;
  private final double kPercentTolerance = 3;

  /**
   * Add your docs here.
   */
  public Lift() {
    // Intert a subsystem name and PID values here
    super("Lift", .05, 0, 0, 0, .01);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.

    leftLiftMotor = new CANSparkMax(RobotMap.leftLiftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLiftMotor = new CANSparkMax(RobotMap.rightLiftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftLiftEncoder = new CANEncoder(leftLiftMotor);
    rightLiftEncoder = new CANEncoder(rightLiftMotor);

    leftLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftLiftMotor.setInverted(true);
    rightLiftMotor.setInverted(false);

    setOutputRange(-1, 1);
    setInputRange(minRightLiftEncoderValue - 5, maxRightLiftEncoderValue + 5);
    setAbsoluteTolerance(kAbsoluteTolerance);
    setPercentTolerance(kPercentTolerance);
    enable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LiftCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return getRightLiftEncoder();
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

  public void setPower(double speed) {
    leftLiftMotor.set(speed);
    rightLiftMotor.set(speed);
  }

  public void stopLift() {
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);
  }
  
  public double getLeftLiftEncoder() {
    return leftLiftEncoder.getPosition();
  }

  public double getRightLiftEncoder() {
    return rightLiftEncoder.getPosition();
  }

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
}
