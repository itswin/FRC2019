/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.command.Subsystem;

import jaci.pathfinder.*;

/**
 * Holds all paths for autonomous
 */
public class Paths extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Each trajectory array consists of a left trajectory (index 0) and a right trajectory (index 1)
  public Trajectory[] testTrajectory;

  public Paths() {
    generatePaths();
    loadPaths();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  
  // Generate any paths needed on the fly
  private void generatePaths() {

  }

  // Load all paths from CSVs
  private void loadPaths() {
    File testTrajectoryFile = new File("/home/lvuser/paths/testPath_left_detailed.csv");

    testTrajectory = new Trajectory[2];
    testTrajectory[0] = Pathfinder.readFromCSV(testTrajectoryFile);
  }
}
