/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * Toggle the intake extension on button press
 */
public class ToggleIntakeExtensionCommand extends InstantCommand {
  public ToggleIntakeExtensionCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.m_intake.intakeExtensionState == Intake.kIntakeRetractedVal) {
      Robot.m_intake.extendIntake();
    } else {
      Robot.m_intake.retractIntake();
    }
  }
}
