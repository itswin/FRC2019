/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSP frontIntake;
  private VictorSP liftIntake;

  public Intake() {
    frontIntake = new VictorSP(RobotMap.frontIntakeMotorPort);
    liftIntake = new VictorSP(RobotMap.liftIntakeMotorPort);

    frontIntake.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intake() {
    frontIntake.set(1);
    liftIntake.set(.3);
  }

  public void outtake() {
    frontIntake.set(-1);
    liftIntake.set(-1);
  }

  public void stopIntake() {
    frontIntake.set(0);
    liftIntake.set(0);
  }

  public void setSpeed(double frontIntakeSpeed, double liftIntakeSpeed) {
    frontIntake.set(frontIntakeSpeed);
    liftIntake.set(liftIntakeSpeed);
  }
}
