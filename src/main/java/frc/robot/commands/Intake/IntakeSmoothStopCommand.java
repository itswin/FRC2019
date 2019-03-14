/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * Fakes coast on the intake motors to avoid shooting
 * the ball out when intake motors stop suddenly
 */
public class IntakeSmoothStopCommand extends Command {
  private final double startingIntakeSpeed = Intake.k_liftIntakeSpeed;
  private final double speedDelta = .005;
  private int count;
  private double currentIntakeSpeed;

  public IntakeSmoothStopCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    currentIntakeSpeed = startingIntakeSpeed;
    count = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intake.setSpeed(currentIntakeSpeed, currentIntakeSpeed);

    currentIntakeSpeed += speedDelta;
    count++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (startingIntakeSpeed / speedDelta) < count;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.stopIntake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_intake.stopIntake();
  }
}
