/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PIDDriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import com.revrobotics.*;

import frc.robot.Robot;

public class SwitchMotorTypeCommand extends Command {
  public SwitchMotorTypeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_pidDriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_pidDriveTrain.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
      Robot.m_pidDriveTrain.changeIdleMode(CANSparkMax.IdleMode.kCoast);
      System.out.println("Current Idle Mode: Coast");
    } else {
      Robot.m_pidDriveTrain.changeIdleMode(CANSparkMax.IdleMode.kBrake);
      System.out.println("Current Idle Mode: Brake");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
