/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveState;

public class AutoForward extends Command {
  private double forwardSpeed;
  public AutoForward(double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);

    // Prevent user error; make sure the robot actually goes forward
    if(speed < 0) {
      speed *= -1;
    }
    forwardSpeed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.driveState = DriveState.kAuto;
    Robot.m_driveTrain.setInputAutoSpeeds(forwardSpeed, 0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_driveTrain.driveState != DriveState.kAuto;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.driveState = DriveState.kManual;
    Robot.m_driveTrain.setInputAutoSpeeds(0, 0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.driveState = DriveState.kManual;
    Robot.m_driveTrain.setInputAutoSpeeds(0, 0, 0);
  }
}
