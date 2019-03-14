/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PIDs;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import frc.robot.subsystems.DriveTrain;
/**
 * Grabs horizontal displacement from limelight network tables
 */
public class HorizontalDistancePIDSource implements PIDSource {
    PIDSourceType sourceType;

    public HorizontalDistancePIDSource() {
        sourceType = PIDSourceType.kDisplacement;
    }
    
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        sourceType = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    @Override
    public double pidGet() {
        double xDisplacement = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double contourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        
        // Avoid divide by 0 errors
        if(contourArea == 0) {
            contourArea = 1;
        }
        double error = -xDisplacement / Math.sqrt(contourArea);
        double sign = Math.signum(error);
        // System.out.println("Horizontal Source: " + -xDisplacement / contourArea);
        // Smaller movements at a closer distance (when the contour is larger)
        // Square root to be less sensitive
        return sign * Math.sqrt(error * sign) - DriveTrain.kHorizontalSetpoint;
	}
}