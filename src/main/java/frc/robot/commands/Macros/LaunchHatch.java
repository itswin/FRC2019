/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveTrain.*;
import frc.robot.commands.HatchMechanism.*;
import frc.robot.commands.Lift.*;
import frc.robot.subsystems.Lift;

public class LaunchHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LaunchHatch() {
    addParallel(new ToggleHatchLauncherState());
    addSequential(new AutoBackup(-.3), .4);
    addSequential(new ToggleHatchLauncherState());
    addSequential(new GoToLiftHeight(Lift.kHome));
  }
}