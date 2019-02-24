/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ClearSchedulerCommand extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ClearSchedulerCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    // Require every subsystem to clear all commands
    requires(Robot.m_pidDriveTrain);
    requires(Robot.m_lift);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }

}
