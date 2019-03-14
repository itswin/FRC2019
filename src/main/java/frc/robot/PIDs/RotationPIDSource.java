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

/**
 * Add your docs here.
 */
public class RotationPIDSource implements PIDSource {
    PIDSourceType sourceType;

    public RotationPIDSource() {
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
        // double totalContourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        // double biggerContourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(0);
        // double smallerContourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta1").getDouble(0);

        // // System.out.println(biggerContourArea - smallerContourArea);

        // // When the target is farther, you need to rotate more
        // // Area scales quadratically with distance so sqrt is needed
        // double adjustmentScalar = Math.sqrt(Robot.m_driveTrain.kVerticalTarget / totalContourArea);
        // adjustmentScalar = 1;

        // double biggerContourX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
        // double smallerContourX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1").getDouble(0);

        // if(biggerContourX > smallerContourX) {
        //     return adjustmentScalar * (smallerContourArea - biggerContourArea);
        // }

        // return adjustmentScalar * (biggerContourArea - smallerContourArea);

        double[] camTran = new double[6];
        camTran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camTran").getDoubleArray(camTran);
        return 0;
	}
}
