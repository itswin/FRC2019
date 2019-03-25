/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftCommand extends Command {
  private boolean canMove;
  private boolean wasMoving;
  private double kMaxSpeedDeltaPerLoop = .15;
  private double kMaxNegSpeedDeltaPerLoop = .05;
  private final boolean rampRateEnabled = true;

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
    canMove = true;
    double inputSpeed = Robot.m_lift.getInputJoystickSpeed();
    double currentSpeed = Robot.m_lift.getCurrentSpeed();

    if(inputSpeed != 0) {
      // Disables the lift PID if manual input is supplied
      if(!wasMoving) {
        wasMoving = true;
        Robot.m_lift.disable();
      }
    } else if(wasMoving) {
      wasMoving = false;
      canMove = false;
      Robot.m_lift.stopLift();
      Robot.m_lift.setSetpoint(Robot.m_lift.getEncoderAverage());
      Robot.m_lift.enable();
    } else {
      inputSpeed = Robot.m_lift.getInputAutoSpeed();
    }

    // Programatically stops motors at set encoder limits
    if((inputSpeed < 0 && !(Robot.m_lift.canLeftLiftAscend() && Robot.m_lift.canRightLiftAscend())) ||
      (inputSpeed > 0 && !(Robot.m_lift.canLeftLiftDescend() && Robot.m_lift.canRightLiftDescend()))) {
      canMove = false;
      Robot.m_lift.stopLift();
    }

    // Limit the rate you can change speed for all directions
    if(rampRateEnabled && inputSpeed > currentSpeed + kMaxSpeedDeltaPerLoop) {
      currentSpeed += kMaxSpeedDeltaPerLoop;
    } else if(rampRateEnabled && currentSpeed < 0 && inputSpeed < currentSpeed - kMaxNegSpeedDeltaPerLoop) {
      // Use a different delta when the lift is falling and accelerating downwards
      currentSpeed -= kMaxNegSpeedDeltaPerLoop;
    } else if(rampRateEnabled && inputSpeed < currentSpeed - kMaxSpeedDeltaPerLoop) {
      // Use the normal delta when the lift is falling and accelerating upwards
      currentSpeed -= kMaxSpeedDeltaPerLoop;
    } else {
      currentSpeed = inputSpeed;
    }

    // canMove turns false whenever the lift HAS to stop
    if(!canMove) {
      currentSpeed = 0;
    }

    Robot.m_lift.setCurrentSpeed(currentSpeed);

    // System.out.println(currentSpeed);
    Robot.m_lift.setPower(Robot.m_lift.getCurrentSpeed());
    // Robot.m_lift.setLeftPower(Robot.m_lift.getCurrentSpeed());
    
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
