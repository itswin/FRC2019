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

public class CenterRotationOnVisionTarget extends Command {
  private double deltaSetpoint = 0;
  private boolean isSetpointSet = false;
  public CenterRotationOnVisionTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_cR);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // System.out.println("Centering rotation");
    // Robot.m_driveTrain.rotationPIDController.disable();
    // Robot.m_driveTrain.LLrotationPIDController.enable();
    // Robot.m_driveTrain.driveState = DriveTrain.DriveState.kAuto;

    System.out.println("Centering rotation");
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kAuto;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double[] camTran = new double[6];
    camTran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(camTran);

    deltaSetpoint = camTran[4];

    if(deltaSetpoint != Double.NaN && Robot.m_driveTrain.rotationPIDController.onTarget()) {
      Robot.m_driveTrain.rotationPIDController.setSetpoint((Robot.getComparedYaw() + deltaSetpoint) % 180);
      isSetpointSet = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return Robot.m_driveTrain.LLrotationPIDController.onTarget() || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto;
    // return Robot.m_driveTrain.centeringPIDsOnTarget() || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto;
    boolean isTargetVisible = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    return isSetpointSet || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto || !isTargetVisible;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Finished centering rotation");
    // Robot.m_driveTrain.LLrotationPIDController.reset();
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    // Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.m_navX.getYaw());
    // Robot.m_driveTrain.rotationPIDController.enable();

    // Robot.m_driveTrain.rotationPIDController.setSetpoint((Robot.m_driveTrain.rotationPIDController.getSetpoint() + deltaSetpoint) % 180);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Centering rotation interrupted");
    // Robot.m_driveTrain.LLrotationPIDController.reset();
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    // Robot.m_driveTrain.rotationPIDController.setSetpoint(Robot.m_navX.getYaw());
    // Robot.m_driveTrain.rotationPIDController.enable();
    // Robot.m_driveTrain.rotationPIDController.setSetpoint((Robot.m_navX.getYaw() + deltaSetpoint) % 180);
  }
}
