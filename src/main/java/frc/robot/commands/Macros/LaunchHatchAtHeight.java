/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.DriveTrain.*;
import frc.robot.commands.HatchMechanism.ToggleHatchLauncherState;
import frc.robot.commands.Lift.*;
import frc.robot.subsystems.Lift;

public class LaunchHatchAtHeight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LaunchHatchAtHeight(double height) {
    addSequential(new GoToLiftHeight(height));
    addSequential(new WaitCommand(.25));
    addSequential(new WaitForLift());
    addSequential(new AutoForward(.2), .35);
    addSequential(new WaitCommand(.25));
    addParallel(new ToggleHatchLauncherState());
    addSequential(new AutoBackup(-.3), .4);
    addSequential(new ToggleHatchLauncherState());
    addSequential(new GoToLiftHeight(Lift.kHome));
  }
}
