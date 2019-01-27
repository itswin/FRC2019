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

  /**
   * Add your docs here.
   */
  public Lift() {
    // Intert a subsystem name and PID values here
    super("Lift", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.

    leftLiftMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightLiftMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftLiftEncoder = new CANEncoder(leftLiftMotor);
    rightLiftEncoder = new CANEncoder(rightLiftMotor);

    leftLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftLiftMotor.setInverted(true);
    rightLiftMotor.setInverted(false);
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
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
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
}
