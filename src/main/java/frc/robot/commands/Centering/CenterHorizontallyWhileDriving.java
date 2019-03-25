/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Centering;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class CenterHorizontallyWhileDriving extends Command {
  public CenterHorizontallyWhileDriving() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_cH);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kAutoHorizontal;
    Robot.m_driveTrain.horizontalPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Robot.m_driveTrain.horizontalPIDController.onTarget() || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto;
    // Command is meant to be run while button is held so only quit if the target is not seen
    boolean isTargetVisible = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    return !isTargetVisible;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    Robot.m_driveTrain.horizontalPIDController.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    Robot.m_driveTrain.horizontalPIDController.reset();
  }
}
