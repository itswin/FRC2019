/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainBaseCommand extends Command {
  private boolean wasMoving;
  private final double kMaxSpeedDeltaPerLoop = .1;
  private final boolean rampRateEnabled = false;

  public DriveTrainBaseCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    wasMoving = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double inputSpeed = 0;
    double inputStrafeSpeed = 0;
    double inputRotationSpeed = Robot.m_driveTrain.getInputJoystickRotationSpeed();

    switch(Robot.m_driveTrain.driveState) {
      case kManual:
        inputSpeed = Robot.m_driveTrain.getInputJoystickSpeed();
        inputStrafeSpeed = Robot.m_driveTrain.getInputJoystickStrafeSpeed();
        break;
      case kAuto:
        inputSpeed = Robot.m_driveTrain.getInputAutoSpeed();
        inputStrafeSpeed = Robot.m_driveTrain.getInputAutoStrafeSpeed();
        break;
      case kAutoHorizontal:
        inputStrafeSpeed = Robot.m_driveTrain.getInputAutoStrafeSpeed();
        break;
    }

    double currentSpeed = Robot.m_driveTrain.getCurrentSpeed();
    double currentStrafeSpeed = Robot.m_driveTrain.getCurrentStrafeSpeed();
    double currentRotationSpeed = Robot.m_driveTrain.getCurrentRotationSpeed();

    if(inputRotationSpeed != 0) {
      // Disables the rotation PID if there is a rotation input
      if(!wasMoving) {
        wasMoving = true;
        Robot.m_driveTrain.rotationPIDController.disable();
      }
    } else if(wasMoving) {
      // Reenables the PID if the robot was just manually being rotated and isn't anymore
      // Waits for the rotation momentum to stop
      if(Math.abs(Robot.m_navX.getRate()) < DriveTrain.rotationThreshold) {
        wasMoving = false;
        Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.getComparedYaw());
        Robot.m_driveTrain.rotationPIDController.enable();
      }
    } else {
      // Only give rotation correction if robot isn't rotating manually
      inputRotationSpeed = Robot.m_driveTrain.getInputAutoRotationSpeed();
    }
    
    // Limit the rate you can change speed for all directions
    // Not used right now, still has problems
    if(rampRateEnabled && inputSpeed > currentSpeed + kMaxSpeedDeltaPerLoop) {
      currentSpeed += kMaxSpeedDeltaPerLoop;
    } else if(rampRateEnabled && inputSpeed < currentSpeed - kMaxSpeedDeltaPerLoop) {
      currentSpeed -= kMaxSpeedDeltaPerLoop;
    } else {
      currentSpeed = inputSpeed;
    }

    if(rampRateEnabled && inputStrafeSpeed > currentStrafeSpeed + kMaxSpeedDeltaPerLoop) {
      currentStrafeSpeed += kMaxSpeedDeltaPerLoop;
    } else if(rampRateEnabled && inputStrafeSpeed < currentStrafeSpeed - kMaxSpeedDeltaPerLoop) {
      currentStrafeSpeed -= kMaxSpeedDeltaPerLoop;
    } else {
      currentStrafeSpeed = inputStrafeSpeed;
    }

    if(rampRateEnabled && inputRotationSpeed > currentRotationSpeed + kMaxSpeedDeltaPerLoop) {
      currentRotationSpeed += kMaxSpeedDeltaPerLoop;
    } else if(rampRateEnabled && inputRotationSpeed < currentRotationSpeed - kMaxSpeedDeltaPerLoop) {
      currentRotationSpeed -= kMaxSpeedDeltaPerLoop;
    } else {
      currentRotationSpeed = inputRotationSpeed;
    }
    
    Robot.m_driveTrain.setCurrentSpeeds(currentSpeed, currentStrafeSpeed, currentRotationSpeed);

    // Robot oriented driving
    Robot.m_driveTrain.drive(Robot.m_driveTrain.getCurrentSpeed(), Robot.m_driveTrain.getCurrentStrafeSpeed(), Robot.m_driveTrain.getCurrentRotationSpeed());

    // Field oriented driving
    // Robot.m_driveTrain.drive(Robot.m_driveTrain.getCurrentSpeed(), Robot.m_driveTrain.getCurrentStrafeSpeed(), Robot.m_driveTrain.getCurrentRotationSpeed(), -Robot.m_navX.getYaw());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.stop();
  }
}