/**
 * Controller class for the driver
 * Instantiates buttons to facilitate writing code for attaching buttons to commands
 * 
 * Built for Logitech Gamepad F310 on X mode
 */

package frc.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LogitechController {
    
    private Joystick m_stick;
    private final static double deadband = .08;

    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;
    public JoystickButton yButton;
    public JoystickButton leftBumperButton;
    public JoystickButton rightBumperButton;
    public JoystickButton backButton;
    public JoystickButton startButton;
    public JoystickButton leftJoystickButton;
    public JoystickButton rightJoystickButton;
    public POVMultiButton povDownButton;
    public POVMultiButton povRightButton;
    public POVMultiButton povUpButton;
    public POVMultiButton povLeftButton;

    public LogitechController(int port) {
        m_stick = new Joystick(port);

        aButton = new JoystickButton(m_stick, 1);
        bButton = new JoystickButton(m_stick, 2);
        xButton = new JoystickButton(m_stick, 3);
        yButton = new JoystickButton(m_stick, 4);
        leftBumperButton = new JoystickButton(m_stick, 5);
        rightBumperButton = new JoystickButton(m_stick, 6);
        backButton = new JoystickButton(m_stick, 7);
        startButton = new JoystickButton(m_stick, 8);
        leftJoystickButton = new JoystickButton(m_stick, 9);
        rightJoystickButton = new JoystickButton(m_stick, 10);
        povDownButton = new POVMultiButton(m_stick, 180, 0);
        povRightButton = new POVMultiButton(m_stick, 90, 0);
        povUpButton = new POVMultiButton(m_stick, 0, 0);
        povLeftButton = new POVMultiButton(m_stick, 270, 0);
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