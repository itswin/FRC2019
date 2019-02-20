/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PIDDriveTrainCommand extends Command {
  private boolean wasMoving;
  private final double rotationScalar = .75;

  private double currentSpeed = 0;
  private double currentStrafe = 0;
  private double currentRotation = 0;
  private final double kMaxSpeedDeltaPerLoop = .1;

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
    double scalar = 1 - .5 * (Robot.m_lift.getRightLiftEncoder() / Robot.m_lift.getMaxRightLiftEncoderValue());

    double speed = -Robot.m_oi.driveController.getLeftYAxis() * scalar;
    double strafe = Robot.m_oi.driveController.getLeftXAxis() * scalar;
    double rotation = Robot.m_oi.driveController.getRightXAxis() * scalar * rotationScalar;

    // Square the values to make driving less sensitive
    // speed = speed * speed * Math.signum(speed);
    // strafe = strafe * strafe * Math.signum(strafe);
    rotation = rotation*rotation * Math.signum(rotation);

    if(rotation != 0) {
      if(!wasMoving) {
        wasMoving = true;
        Robot.m_pidDriveTrain.disable();
      }
    } else {
      if(wasMoving) {
        wasMoving = false;
        Robot.m_pidDriveTrain.setSetpoint360(Robot.m_navX.getAngle());
        Robot.m_pidDriveTrain.enable();
      }
      rotation = Robot.m_pidDriveTrain.getRotationSpeed();
    }

    /*
    // Limit the rate you can change speed for all directions
    if(speed > currentSpeed + kMaxSpeedDeltaPerLoop) {
      currentSpeed += kMaxSpeedDeltaPerLoop;
    } else if(speed < currentSpeed - kMaxSpeedDeltaPerLoop) {
      currentSpeed -= kMaxSpeedDeltaPerLoop;
    } else {
      currentSpeed = speed;
    }

    if(strafe > currentStrafe + kMaxSpeedDeltaPerLoop) {
      currentStrafe += kMaxSpeedDeltaPerLoop;
    } else if(strafe < currentStrafe - kMaxSpeedDeltaPerLoop) {
      currentStrafe -= kMaxSpeedDeltaPerLoop;
    } else {
      currentStrafe = speed;
    }

    if(rotation > currentRotation + kMaxSpeedDeltaPerLoop) {
      currentRotation += kMaxSpeedDeltaPerLoop;
    } else if(rotation < currentRotation - kMaxSpeedDeltaPerLoop) {
      currentRotation -= kMaxSpeedDeltaPerLoop;
    } else {
      currentRotation = rotation;
    } */
    

    currentSpeed = speed;
    currentStrafe = strafe;
    currentRotation = rotation;

    // System.out.println("Rotation speed: " + rotation);

    // With angle
    Robot.m_pidDriveTrain.drive(currentSpeed, currentStrafe, currentRotation);

    System.out.println(currentSpeed + ", " + currentStrafe + ", " + currentRotation);

    // Without angle
    // Robot.m_pidDriveTrain.drive(speed, strafe, rotation, -Robot.m_navX.getAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    currentSpeed = 0;
    currentStrafe = 0;
    currentRotation = 0;
    Robot.m_pidDriveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    currentSpeed = 0;
    currentStrafe = 0;
    currentRotation = 0;
    Robot.m_pidDriveTrain.stopDriveTrain();
  }
}
