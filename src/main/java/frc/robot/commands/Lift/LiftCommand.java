/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

public class LiftCommand extends Command {
  private boolean wasMoving;
  public LiftCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_lift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    wasMoving = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(OI.driveController.getRightTrigger() > 0) {
      // Right trigger to ascend
      // Programatically stops motors at set encoder limits
      if(Robot.m_lift.canLeftLiftAscend() && Robot.m_lift.canRightLiftAscend()) {
        // Disables the lift PID if manual input is supplied
        if(!wasMoving) {
          wasMoving = true;
          Robot.m_lift.disable();
        }
        Robot.m_lift.setPower(-OI.driveController.getRightTrigger());
      } else {
        Robot.m_lift.stopLift();
      }
    } else if(OI.driveController.getLeftTrigger() > 0) {
      // Left trigger to descend
      if(Robot.m_lift.canLeftLiftDescend() && Robot.m_lift.canRightLiftDescend()) {
        if(!wasMoving) {
          wasMoving = true;
          Robot.m_lift.disable();
        }
        Robot.m_lift.setPower(OI.driveController.getLeftTrigger());
      } else {
        Robot.m_lift.stopLift();
      }
    } else {
      Robot.m_lift.stopLift();
      // Reenables PID if no input is supplied
      if(wasMoving) {
        wasMoving = false;
        Robot.m_lift.enable();
        Robot.m_lift.setSetpoint(Robot.m_lift.getEncoderAverage());
      }
    }

    // Resets encoders when limit switches are hit
    if(Robot.m_lift.getLeftLimitSwitchVal() && Robot.m_lift.getRightLimitSwitchVal()) {
      Robot.m_lift.setEncoderComparisons(Robot.m_lift.getTrueLeftLiftEncoder(), Robot.m_lift.getTrueRightLiftEncoder());
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
    Robot.m_lift.stopLift();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_lift.stopLift();
  }
}
