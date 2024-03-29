/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.NavX;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ResetNavX extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ResetNavX() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_navX.reset();
    Robot.m_driveTrain.zeroAngle = 0;
    Robot.m_driveTrain.rotationPIDController.reset();
    Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.getComparedYaw());
    Robot.m_driveTrain.rotationPIDController.enable();
  }

}
