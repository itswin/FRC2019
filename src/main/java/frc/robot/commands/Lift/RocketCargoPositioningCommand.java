/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Lift;
import frc.robot.Robot;
import frc.robot.commands.Macros.*;
import frc.robot.commands.NavX.ResetNavX;

public class RocketCargoPositioningCommand extends Command {
  private boolean povUpPressed;
  private boolean povRightPressed;
  private boolean povDownPressed;
  private boolean povUpLastPressed;
  private boolean povRightLastPressed;
  private boolean povDownLastPressed;
  
  // Not used for positioning actually... this command just runs when the A button is pressed 
  // so I'm stealing its functionality to bind another command to A and POV Left
  private boolean povLeftPressed;
  private boolean povLeftLastPressed;

  public RocketCargoPositioningCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.m_lift);
    requires(Robot.m_aButton);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    povUpPressed = false;
    povRightPressed = false;
    povDownPressed = false;
    povUpLastPressed = false;
    povRightLastPressed = false;
    povDownLastPressed = false;
    povLeftPressed = false;
    povLeftLastPressed = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    povUpPressed = Robot.m_oi.driveController.povUpButton.get();
    povRightPressed = Robot.m_oi.driveController.povRightButton.get();
    povDownPressed = Robot.m_oi.driveController.povDownButton.get();
    povLeftPressed = Robot.m_oi.driveController.povLeftButton.get();

    // Uses joystick hat and A button to send lift to cargo setpoints
    if(povUpPressed && !povUpLastPressed) {
      Scheduler.getInstance().add(new ShootCargoAtHeight(Lift.kThirdRocketCargoHole));
    } else if(povRightPressed && !povRightLastPressed) {
      Scheduler.getInstance().add(new ShootCargoAtHeight(Lift.kSecondRocketCargoHole));
    } else if(povDownPressed && !povDownLastPressed) {
      Scheduler.getInstance().add(new ShootCargoAtHeight(Lift.kFirstRocketCargoHole));
    } else if(povLeftPressed && !povLeftLastPressed) {
      Scheduler.getInstance().add(new ResetNavX());
    }

    povUpLastPressed = povUpPressed;
    povRightLastPressed = povRightPressed;
    povDownLastPressed = povDownPressed;
    povLeftLastPressed = povLeftPressed;
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
}
