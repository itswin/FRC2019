/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Lift;
import frc.robot.Robot;

public class RocketHatchPositioningCommand extends Command {
  public RocketHatchPositioningCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.m_lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Uses joystick hat to send lift to hatch setpoints
    if(Robot.m_oi.driveController.povUpButton.get()) {
      Robot.m_lift.setSetpoint(Lift.kThirdRocketHatch);
    } else if(Robot.m_oi.driveController.povRightButton.get()) {
      Robot.m_lift.setSetpoint(Lift.kSecondRocketHatch);
    } else if(Robot.m_oi.driveController.povDownButton.get()) {
      Robot.m_lift.setSetpoint(Lift.kHome);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
