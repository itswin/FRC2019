/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class GoToLiftHeight extends InstantCommand {
  private double setpoint;
  /**
   * Add your docs here.
   */
  public GoToLiftHeight(double goal) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setpoint = goal;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_lift.setSetpoint(setpoint);
  }

}
