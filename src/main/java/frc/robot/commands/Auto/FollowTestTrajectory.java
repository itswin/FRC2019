/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import java.io.File;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Pathfinder;

public class FollowTestTrajectory extends Command {

  private final String kLeftTrajFileName = "/home/lvuser/paths/testPath_left_detailed.csv";
  private final String kRightTrajFileName = "/home/lvuser/paths/testPath_right_detailed.csv";
  private Trajectory leftTraj, rightTraj;
  private EncoderFollower leftFollower, rightFollower;
  private Notifier autoNotifier;

  public FollowTestTrajectory() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_pidDriveTrain);

    File leftFile = new File(kLeftTrajFileName);
    File rightFile = new File(kRightTrajFileName);
    
    leftTraj = Pathfinder.readFromCSV(leftFile);
    rightTraj = Pathfinder.readFromCSV(rightFile);

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    leftFollower.configureEncoder(Robot.m_pidDriveTrain.getLeftEncoderAverage(), Robot.m_pidDriveTrain.kPulsesPerRevolution, Robot.m_pidDriveTrain.kWheel_diameter, Robot.m_pidDriveTrain.kDistanceScaler);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  
  class FollowTrajectory implements Runnable {
    @Override
    public void run() {
      if (!leftFollower.isFinished() || !rightFollower.isFinished()) {
        double l = leftFollower.calculate((int)Robot.m_pidDriveTrain.getLeftEncoderAverage());
        double r = rightFollower.calculate((int)Robot.m_pidDriveTrain.getRightEncoderAverage());
  
        double gyro_heading = Robot.m_navX.getAngle();
        double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
  
        double angleDifference = 0.8 * (-1.0 / 80.0) * Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

        System.out.println(l + turn);
  
        Robot.m_pidDriveTrain.setLeftSpeed(l + turn);
        Robot.m_pidDriveTrain.setRightSpeed(r - turn);
      } else {
        Robot.m_pidDriveTrain.setLeftSpeed(0);
        Robot.m_pidDriveTrain.setRightSpeed(0);
        autoNotifier.stop();
      }
    }
  }
}