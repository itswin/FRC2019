/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HabMechanism;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.HabMechanism;

/**
 * Add your docs here.
 */
public class ToggleBackPistons extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleBackPistons() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.m_habMechanism.getBackPistonExtensionState() == HabMechanism.backSolenoidRetractedVal) {
      Robot.m_habMechanism.extendBackPistons();
      System.out.println("Equals");
    } else {
      Robot.m_habMechanism.retractBackPistons();
      System.out.println("Does not equal");
    }
  }

}
