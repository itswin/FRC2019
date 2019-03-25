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
 * Add your docs here.
 */
public class CargoRotationPIDSource implements PIDSource {
    private PIDSourceType sourceType;

    public CargoRotationPIDSource() {
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
        error = sign * Math.sqrt(error * sign);
        // System.out.println("Horizontal Source: " + error);
        // Smaller movements at a closer distance (when the contour is larger)
        // Square root to be less sensitive
        return error - DriveTrain.kCargoRotationSetpoint;
    }
}