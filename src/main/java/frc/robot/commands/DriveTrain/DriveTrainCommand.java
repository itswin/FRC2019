/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveTrainCommand extends Command {
  public DriveTrainCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Scale driving speed when lift is up
    double scalar = 1 - .5 * (Robot.m_lift.getRightLiftEncoder() / Robot.m_lift.getMaxRightLiftEncoderValue());

    double speed = -Robot.m_oi.driveController.getLeftYAxis() * scalar;
    double strafe = Robot.m_oi.driveController.getLeftXAxis() * scalar;
    double rotation = Robot.m_oi.driveController.getRightXAxis() * scalar;

    // Square rotation to make turning less sensitive
    Robot.m_driveTrain.drive(speed, strafe, Math.signum(rotation)*rotation*rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.stopDriveTrain();
  }
}
