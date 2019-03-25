/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HabMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid frontPistonSolenoid;
  private Solenoid backPistonSolenoid;
  public static final boolean frontSolenoidRetractedVal = false;
  public static final boolean backSolenoidRetractedVal = false;

  public HabMechanism() {
    frontPistonSolenoid = new Solenoid(RobotMap.habFrontSolenoidPort);
    backPistonSolenoid = new Solenoid(RobotMap.habBackSolenoidPort);

    retractFrontPistons();
    retractBackPistons();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean getFrontPistonExtensionState() {
    return frontPistonSolenoid.get();
  }

  public boolean getBackPistonExtensionState() {
    return backPistonSolenoid.get();
  }

  public void retractFrontPistons() {
    frontPistonSolenoid.set(frontSolenoidRetractedVal);
  }

  public void retractBackPistons() {
    System.out.println("Retracting");
    backPistonSolenoid.set(backSolenoidRetractedVal);
  }
  
  public void extendFrontPistons() {
    frontPistonSolenoid.set(!frontSolenoidRetractedVal);
  }
  
  public void extendBackPistons() {
    System.out.println("Extending");
    backPistonSolenoid.set(!backSolenoidRetractedVal);
  }
}
