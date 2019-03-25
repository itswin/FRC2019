/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Centering;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateToCargo extends Command {
  public RotateToCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.rotationPIDController.reset();
    Robot.m_driveTrain.cargoRotationPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.getComparedYaw());
    Robot.m_driveTrain.rotationPIDController.enable();
    Robot.m_driveTrain.cargoRotationPIDController.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.getComparedYaw());
    Robot.m_driveTrain.rotationPIDController.enable();
    Robot.m_driveTrain.cargoRotationPIDController.reset();
  }
}
