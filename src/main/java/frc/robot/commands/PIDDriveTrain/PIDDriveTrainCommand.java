/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PIDDriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Base command for the PIDDriveTrain
 * Sets power to the motors from the input speeds updated in periodics
 */
public class PIDDriveTrainCommand extends Command {
  private boolean wasMoving;
  private final double rotationScalar = .75;
  private final double kMaxSpeedDeltaPerLoop = .1;

  private final boolean rampRateEnabled = false;

  public PIDDriveTrainCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_pidDriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    wasMoving = false;
    Robot.m_pidDriveTrain.setSetpoint360(Robot.m_navX.getAngle());
    Robot.m_pidDriveTrain.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double inputSpeed = Robot.m_pidDriveTrain.getInputSpeed();
    double inputStrafeSpeed = Robot.m_pidDriveTrain.getInputStrafeSpeed();
    double inputRotationSpeed = Robot.m_pidDriveTrain.getInputRotationSpeed();

    double currentSpeed = Robot.m_pidDriveTrain.getCurrentSpeed();
    double currentStrafeSpeed = Robot.m_pidDriveTrain.getCurrentStrafeSpeed();
    double currentRotationSpeed = Robot.m_pidDriveTrain.getCurrentRotationSpeed();

    if(inputRotationSpeed != 0) {
      // Disables the rotation PID if there is a rotation input
      if(!wasMoving) {
        wasMoving = true;
        Robot.m_pidDriveTrain.disable();
      }
    } else {
      // Reenables the PID if the robot was just manually being rotated and isn't anymore
      if(wasMoving) {
        wasMoving = false;
        Robot.m_pidDriveTrain.setSetpoint360(Robot.m_navX.getAngle());
        Robot.m_pidDriveTrain.enable();
      }

      inputRotationSpeed = Robot.m_pidDriveTrain.getPidRotationSpeed();
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
    
    Robot.m_pidDriveTrain.setCurrentSpeeds(currentRotationSpeed, currentStrafeSpeed, currentRotationSpeed);

    // Robot oriented driving
    Robot.m_pidDriveTrain.drive(Robot.m_pidDriveTrain.getCurrentSpeed(), Robot.m_pidDriveTrain.getCurrentStrafeSpeed(), Robot.m_pidDriveTrain.getCurrentRotationSpeed());

    // Field oriented driving
    // Robot.m_pidDriveTrain.drive(Robot.m_pidDriveTrain.getCurrentSpeed(), Robot.m_pidDriveTrain.getCurrentStrafeSpeed(), Robot.m_pidDriveTrain.getCurrentRotationSpeed(), -Robot.m_navX.getAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_pidDriveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_pidDriveTrain.stopDriveTrain();
  }
}
