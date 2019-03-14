/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchMechanism;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.HatchMechanism;

/**
 * Toggles the hatch launcher on button press
 */
public class ToggleHatchLauncherState extends InstantCommand {
  public ToggleHatchLauncherState() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_hatchMechanism);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.m_hatchMechanism.getHatchLauncherExtensionState() == HatchMechanism.hatchLauncherRetractedValue) {
      Robot.m_hatchMechanism.extendHatchLauncher();
    } else {
      Robot.m_hatchMechanism.retractHatchLauncher();
    }
  }

}
