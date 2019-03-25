/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Intake extends Subsystem {
  private VictorSP frontIntake;
  private VictorSP liftIntake;
  private DoubleSolenoid intakeSolenoid;

  // Have constants to compare to so magic values aren't floating around
  public static final DoubleSolenoid.Value kIntakeRetractedVal = DoubleSolenoid.Value.kReverse;
  public static final DoubleSolenoid.Value kIntakeExtendedVal = DoubleSolenoid.Value.kForward;
  public DoubleSolenoid.Value intakeExtensionState = kIntakeRetractedVal;

  public static final double k_liftIntakeSpeed = -1;
  private double currentFrontIntakeSpeed = 0;
  private double currentLiftIntakeSpeed = 0;

  public Intake() {
    frontIntake = new VictorSP(RobotMap.frontIntakeMotorPort);
    liftIntake = new VictorSP(RobotMap.liftIntakeMotorPort);
    intakeSolenoid = new DoubleSolenoid(RobotMap.intakeSolenoidOnPort, RobotMap.intakeSolenoidOffPort);

    intakeSolenoid.set(intakeExtensionState);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new IntakeBaseCommand());
  }

  @Override
  public void periodic() {
    if(Robot.m_lift.getEncoderAverage() > Lift.kFirstRocketCargoHole && 
        intakeExtensionState == kIntakeExtendedVal) {
      Robot.m_intake.retractIntake();
    }

    setPower(currentFrontIntakeSpeed, currentLiftIntakeSpeed);
  }

  // ********** Motor methods ********** //
  public void intake() {
    currentFrontIntakeSpeed = -1;
    currentLiftIntakeSpeed = k_liftIntakeSpeed;
  }

  public void outtake() {
    currentFrontIntakeSpeed = 1;
    currentLiftIntakeSpeed = 1;
  }

  public void stopIntake() {
    currentFrontIntakeSpeed = 0;
    currentLiftIntakeSpeed = 0;
  }

  public void setCurrentIntakeSpeeds(double frontIntakeSpeed, double liftIntakeSpeed) {
    currentFrontIntakeSpeed = frontIntakeSpeed;
    currentLiftIntakeSpeed = liftIntakeSpeed;
  }

  public void setPower(double frontIntakeSpeed, double liftIntakeSpeed) {
    frontIntake.set(frontIntakeSpeed);
    liftIntake.set(liftIntakeSpeed);
  }

  // ********** Solenoid methods ********** //
  public void extendIntake() {
    intakeExtensionState = kIntakeExtendedVal;
    intakeSolenoid.set(intakeExtensionState);
  }

  public void retractIntake() {
    intakeExtensionState = kIntakeRetractedVal;
    intakeSolenoid.set(intakeExtensionState);
  }
}
