/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PIDs;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrainStrafePIDOutput implements PIDOutput {
    private double threshold = .05;

    @Override
    public void pidWrite(double output) {
        double sign = Math.signum(output);
        // Don't let the robot jiter (quick changes from -.2 to .2)
        if(Math.abs(output) < threshold) {
            sign = 0;
            output = 0;
        }

        Robot.m_driveTrain.setInputAutoStrafeSpeed(sign * Robot.m_driveTrain.kHorizontalBaseline + output);
        System.out.println(sign * Robot.m_driveTrain.kHorizontalBaseline + output);
    }
}
