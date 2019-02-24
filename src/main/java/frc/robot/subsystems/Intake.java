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
import frc.robot.RobotMap;
import frc.robot.commands.Intake.IntakeBaseCommand;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSP frontIntake;
  private VictorSP liftIntake;
  private DoubleSolenoid intakeSolenoid;
  public static final DoubleSolenoid.Value intakeRetractedVal = DoubleSolenoid.Value.kReverse;
  public static final DoubleSolenoid.Value intakeExtendedVal = DoubleSolenoid.Value.kForward;
  public static DoubleSolenoid.Value intakeExtensionState = intakeRetractedVal;

  public static final double k_liftIntakeSpeed = -1;

  public Intake() {
    frontIntake = new VictorSP(RobotMap.frontIntakeMotorPort);
    liftIntake = new VictorSP(RobotMap.liftIntakeMotorPort);
    intakeSolenoid = new DoubleSolenoid(RobotMap.intakeSolenoidOnPort, RobotMap.intakeSolenoidOffPort);

    intakeSolenoid.set(intakeExtensionState);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeBaseCommand());
  }

  public void intake() {
    frontIntake.set(-1);
    liftIntake.set(k_liftIntakeSpeed);
  }

  public void outtake() {
    frontIntake.set(1);
    liftIntake.set(1);
  }

  public void stopIntake() {
    frontIntake.set(0);
    liftIntake.set(0);
  }

  public void setSpeed(double frontIntakeSpeed, double liftIntakeSpeed) {
    frontIntake.set(frontIntakeSpeed);
    liftIntake.set(liftIntakeSpeed);
  }

  public void extendIntake() {
    intakeExtensionState = intakeExtendedVal;
    intakeSolenoid.set(intakeExtensionState);
  }

  public void retractIntake() {
    intakeExtensionState = intakeRetractedVal;
    intakeSolenoid.set(intakeExtensionState);
  }

  public void offIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    intakeExtensionState = DoubleSolenoid.Value.kOff;
  }
}
