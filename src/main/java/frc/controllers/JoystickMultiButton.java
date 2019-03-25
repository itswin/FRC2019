/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Joystick button extension of the MultiButton class
 */
public class JoystickMultiButton extends MultiButton {
    private final GenericHID m_joystick;
    private final int m_buttonNumber;
  
    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
     *                     etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public JoystickMultiButton(GenericHID joystick, int buttonNumber) {
      m_joystick = joystick;
      m_buttonNumber = buttonNumber;
    }
  
    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
      return m_joystick.getRawButton(m_buttonNumber);
    }
}
