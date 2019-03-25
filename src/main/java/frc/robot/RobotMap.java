/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // SparkMAX Ports
  public static final int frontLeftMotorID = 2;
  public static final int backLeftMotorID = 3;
  public static final int frontRightMotorID = 4;
  public static final int backRightMotorID = 5;
  public static final int rightLiftMotorID = 1;
  public static final int leftLiftMotorID = 6;

  // PWM Ports
  public static final int frontIntakeMotorPort = 1;
  public static final int liftIntakeMotorPort = 0;

  // Solenoid Ports
  public static final int intakeSolenoidOnPort = 0;
  public static final int intakeSolenoidOffPort = 1;
  public static final int hatchLauncherSolenoidOnPort = 2;
  public static final int hatchLauncherSolenoidOffPort = 3;
  public static final int hatchExtenderSolenoidOnPort = 4;
  public static final int hatchExtenderSolenoidOffPort = 5;
  public static final int habFrontSolenoidPort = 6;
  public static final int habBackSolenoidPort = 7;
}
