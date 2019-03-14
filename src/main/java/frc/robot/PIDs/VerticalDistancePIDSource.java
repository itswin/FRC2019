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
 * Grabs vertical displacement from limelight network tables
 */
public class VerticalDistancePIDSource implements PIDSource {
    PIDSourceType sourceType;

    public VerticalDistancePIDSource() {
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
        double contourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        return contourArea - DriveTrain.kVerticalTarget;
	}
}
