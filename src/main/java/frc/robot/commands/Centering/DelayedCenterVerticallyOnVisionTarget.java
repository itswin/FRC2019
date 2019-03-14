/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Centering;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DelayedCenterVerticallyOnVisionTarget extends CommandGroup {
  /**
   * Add your docs here.
   */

  public DelayedCenterVerticallyOnVisionTarget() {
    addSequential(new WaitForRotation());
    addSequential(new WaitForHorizontal());
    addSequential(new CenterVerticallyOnVisionTarget());
  }
}
