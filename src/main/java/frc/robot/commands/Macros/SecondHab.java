/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.Centering.*;
import frc.robot.commands.DriveTrain.*;
import frc.robot.commands.HabMechanism.*;

public class SecondHab extends CommandGroup {
  private final double timeToExtend = 1;
  /**
   * Add your docs here.
   */
  public SecondHab() {
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
    // addSequential(new GoToAngle(180));
    addSequential(new AutoBackup(-.1), .1);
    addSequential(new ToggleFrontPistons());
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new AutoForward(.3), .75);
    addSequential(new AutoForward(.15), .75);
    addSequential(new ToggleBackPistons());
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new AutoForward(.15), .2);
    addSequential(new ToggleFrontPistons());
    addSequential(new WaitCommand(timeToExtend));
    addSequential(new AutoForward(.3), .75);
    addSequential(new AutoForward(.15), .75);
    addSequential(new ToggleBackPistons());
  }
}
