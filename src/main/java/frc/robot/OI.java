/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.controllers.LogitechController;
import frc.robot.commands.Centering.*;
import frc.robot.commands.HabMechanism.*;
import frc.robot.commands.HatchMechanism.*;
import frc.robot.commands.Lift.*;
import frc.robot.commands.Macros.*;
import frc.robot.commands.NavX.*;
import frc.robot.subsystems.Lift;
import frc.robot.commands.Intake.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public LogitechController driveController;

  public OI() {
    driveController = new LogitechController(0);

    // Intake
    driveController.leftJoystickButton.whileHeld(new IntakeCommand());
    driveController.leftJoystickButton.whenReleased(new IntakeSmoothStopCommand());
    driveController.leftBumperButton.whileHeld(new OuttakeCommand());
    driveController.leftBumperButton.whenReleased(new StopIntakeCommand());
    driveController.xButton.whenPressed(new ToggleIntakeExtensionCommand());

    // Lift
    driveController.povDownButton.whenPressedWith(new ShootCargoAtHeight(Lift.kFirstRocketCargoHole), driveController.aButton);
    driveController.povRightButton.whenPressedWith(new ShootCargoAtHeight(Lift.kSecondRocketCargoHole), driveController.aButton);
    driveController.povUpButton.whenPressedWith(new ShootCargoAtHeight(Lift.kThirdRocketCargoHole), driveController.aButton);
    driveController.povDownButton.whenNotPressedWith(new LaunchHatchAtHeight(Lift.kHome), driveController.aButton);
    driveController.povRightButton.whenNotPressedWith(new LaunchHatchAtHeight(Lift.kSecondRocketHatch), driveController.aButton);
    driveController.povUpButton.whenNotPressedWith(new LaunchHatchAtHeight(Lift.kThirdRocketHatch), driveController.aButton);
    driveController.povLeftButton.whenNotPressedWith(new ShootCargoAtHeight(Lift.kCargoShip), driveController.aButton);

    // Hatch Mechanism
    driveController.bButton.whenPressed(new GrabHatchFromStation());
    driveController.yButton.whenPressed(new LaunchHatch());
    
    // Vision tracking
    driveController.rightJoystickButton.whileActive(new LineUpWhileDriving()); // was whileActive
    driveController.rightJoystickButton.whenReleased(new CancelLineupAndTurnOffLEDs());

    driveController.rightBumperButton.whenPressed(new FollowCargo());
    driveController.rightBumperButton.whenReleased(new StopFollowingCargo());
    
    // Hab
    driveController.backButton.whenPressed(new SecondHab());
    driveController.startButton.whenPressed(new ToggleBackPistons());

    // Miscellaneous
    driveController.povLeftButton.whenPressedWith(new ResetNavX(), driveController.aButton);
  }
}