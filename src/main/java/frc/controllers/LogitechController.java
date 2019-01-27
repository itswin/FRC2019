/**
 * Controller class for the driver
 * Abstracts buttons to facilitate writing code for attaching buttons to commands
 */

package frc.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LogitechController {
    
    private Joystick m_stick;
    private final static double deadband = .05;

    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;
    public JoystickButton yButton;
    public JoystickButton leftBumperButton;
    public JoystickButton rightBumperButton;

    public LogitechController(int port) {
        m_stick = new Joystick(port);

        aButton = new JoystickButton(m_stick, 1);
        bButton = new JoystickButton(m_stick, 2);
        xButton = new JoystickButton(m_stick, 3);
        yButton = new JoystickButton(m_stick, 4);
        leftBumperButton = new JoystickButton(m_stick, 5);
        rightBumperButton = new JoystickButton(m_stick, 6);
    }

    public double getLeftXAxis() {
        double val = m_stick.getRawAxis(0);
        return handleDeadband(val);
    }

    public double getLeftYAxis() {
        double val = m_stick.getRawAxis(1);
        return handleDeadband(val);
    }

    public double getRightXAxis() {
        double val = m_stick.getRawAxis(4);
        return handleDeadband(val);
    }

    public double getRightYAxis() {
        double val = m_stick.getRawAxis(5);
        return handleDeadband(val);
    }

    public double getRightTrigger() {
        double val = m_stick.getRawAxis(2);
        return handleDeadband(val);
    }

    public double getLeftTrigger() {
        double val = m_stick.getRawAxis(3);
        return handleDeadband(val);
    }

    private double handleDeadband(double num) {
        if(Math.abs(num) < deadband) {
            return 0;
        }

        return num;
    }
}