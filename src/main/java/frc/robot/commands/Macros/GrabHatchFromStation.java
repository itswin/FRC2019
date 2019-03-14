/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import frc.robot.commands.Lift.*;
import frc.robot.commands.HatchMechanism.*;
import frc.robot.commands.DriveTrain.*;

public class GrabHatchFromStation extends CommandGroup {
  public static final double timeToExtend = .5; // in seconds

  /**
   * Add your docs here.
   */
  public GrabHatchFromStation() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    addParallel(new ToggleHatchExtenderState());
    addParallel(new AutoBackup(-.1), timeToExtend);
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new StationHatchHeight());
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new ToggleHatchExtenderState());
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new HomeHeight());
  }
}
