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
import frc.robot.commands.Intake.*;
import frc.robot.commands.Lift.*;
import frc.robot.subsystems.Lift;

public class ShootCargoAtHeight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootCargoAtHeight(double height) {
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
    addParallel(new GoToLiftHeight(height));
    addParallel(new SlowLiftIntake());
    addSequential(new WaitCommand(.25));
    addParallel(new RetractIntake());
    addSequential(new WaitForLift());
    addSequential(new AutoForward(.2), .35);
    addSequential(new WaitCommand(.25));
    addSequential(new OuttakeCommand());
    addSequential(new WaitCommand(.5));
    addParallel(new StopIntakeCommand());
    addSequential(new AutoBackup(-.2), .15);
    addSequential(new GoToLiftHeight(Lift.kHome));
  }
}
