/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.motion;

/**
 * Add your docs here.
 */
public class ProfilePoint {
	
	public double pos = 0;
	public double vel = 0;
	public double acc = 0;
	
	public ProfilePoint(double pos, double vel, double acc) {
		this.pos = pos;
		this.vel = vel;
		this.acc = acc;
	}
	
	public ProfilePoint() {}
}