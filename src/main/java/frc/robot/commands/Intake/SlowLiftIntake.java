/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SlowLiftIntake extends InstantCommand {

  private final double intakeSpeed = .3;

  /**
   * Add your docs here.
   */
  public SlowLiftIntake() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_intake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_intake.setCurrentIntakeSpeeds(0, -intakeSpeed);
  }

}
