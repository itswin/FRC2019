/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.HatchMechanism.*;

public class HatchMechanism extends Subsystem {
  private DoubleSolenoid hatchLauncherSolenoid;
  private DoubleSolenoid hatchExtenderSolenoid;

  // Have constants to compare to so magic values aren't floating around
  public final static DoubleSolenoid.Value hatchLauncherRetractedValue = DoubleSolenoid.Value.kForward;
  public final static DoubleSolenoid.Value hatchLauncherExtendedValue = DoubleSolenoid.Value.kReverse;
  private DoubleSolenoid.Value hatchLauncherExtensionState = hatchLauncherRetractedValue;

  public final static DoubleSolenoid.Value hatchExtenderRetractedValue = DoubleSolenoid.Value.kReverse;
  public final static DoubleSolenoid.Value hatchExtenderExtendedValue = DoubleSolenoid.Value.kForward;
  private DoubleSolenoid.Value hatchExtenderExtensionState = hatchExtenderRetractedValue;

  public HatchMechanism() {
    hatchLauncherSolenoid = new DoubleSolenoid(RobotMap.hatchLauncherSolenoidOnPort, RobotMap.hatchLauncherSolenoidOffPort);
    hatchExtenderSolenoid = new DoubleSolenoid(RobotMap.hatchExtenderSolenoidOnPort, RobotMap.hatchExtenderSolenoidOffPort);

    hatchLauncherSolenoid.set(hatchLauncherExtensionState);
    hatchExtenderSolenoid.set(hatchExtenderExtensionState);
  }

  @Override
  public void initDefaultCommand() {
    // No default command
    // State is updated using buttons mapped to commands in OI
  }

  public DoubleSolenoid.Value getHatchLauncherExtensionState() {
    return hatchLauncherExtensionState;
  }

  public void retractHatchLauncher() {
    hatchLauncherExtensionState = hatchLauncherRetractedValue;
    hatchLauncherSolenoid.set(hatchLauncherExtensionState);
  }

  public void extendHatchLauncher() {
    hatchLauncherExtensionState = hatchLauncherExtendedValue;
    hatchLauncherSolenoid.set(hatchLauncherExtensionState);
  }

  public DoubleSolenoid.Value getHatchExtenderExtensionState() {
    return hatchExtenderExtensionState;
  }

  public void retractHatchExtender() {
    hatchExtenderExtensionState = hatchExtenderRetractedValue;
    hatchExtenderSolenoid.set(hatchExtenderExtensionState);
  }

  public void extendHatchExtender() {
    hatchExtenderExtensionState = hatchExtenderExtendedValue;
    hatchExtenderSolenoid.set(hatchExtenderExtensionState);
  }
}