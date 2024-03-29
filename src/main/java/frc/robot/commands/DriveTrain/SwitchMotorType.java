/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.DriveTrain;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SwitchMotorType extends InstantCommand {
  /**
   * Add your docs here.
   */
  public SwitchMotorType() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.m_driveTrain.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
      Robot.m_driveTrain.changeIdleMode(CANSparkMax.IdleMode.kCoast);
      System.out.println("Current Idle Mode: Coast");
    } else {
      Robot.m_driveTrain.changeIdleMode(CANSparkMax.IdleMode.kBrake);
      System.out.println("Current Idle Mode: Brake");
    }
  }

}
