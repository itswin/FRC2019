/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Provides support to attach one command to two buttons pressed at the same time
 */
public abstract class MultiButton extends Button {
    /**
     * Starts the command whenever two buttons are pressed
     * 
     * @param command the command to start
     * @param button2 the second button that needs to be pressed
     */
    public void whenPressedWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
            private boolean m_button2PressedLast = button2.get();
        
            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();
        
                if (pressed && !m_button2PressedLast && button2Pressed) {
                    // if first button is held and second is pressed
                    command.start();
                } else if(button2Pressed && !m_buttonPressedLast && pressed) {
                    // if second button is held and first is pressed
                    command.start();
                }
        
                m_buttonPressedLast = pressed;
                m_button2PressedLast = button2Pressed;
            }
        }.start();
    }
    
    /**
     * Starts the command whenever the main button is pressed and the second isn't
     * Needed when using whenPressedWith() to bind a command to both buttons pressed at the same time
     * 
     * @param command the command to start
     * @param button2 the second button that can't be pressed
     */
    public void whenNotPressedWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
        
            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();
        
                if (pressed && !m_buttonPressedLast && !button2Pressed) {
                    command.start();
                }
        
                m_buttonPressedLast = pressed;
            }
        }.start();
    }

    /**
     * Constantly starts the given command while two buttons are held
     *
     * {@link Command#start()} will be called repeatedly while both buttons are active, and will be
     * canceled when either becomes inactive
     *
     * @param command the command to start
     * @param button2 the second button that needs to be pressed
     */
    public void whilePressedWith(final Command command, Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
            private boolean m_button2PressedLast = button2.get();
      
            @Override
            public void execute() {
            boolean pressed = get();
            boolean button2Pressed = button2.get();

            if (pressed && button2Pressed) {
                command.start();
            } else if ((m_button2PressedLast && !button2Pressed) || (m_buttonPressedLast && !pressed)) {
                // if either button is released, cancel command
                command.cancel();
            }

              m_buttonPressedLast = pressed;
              m_button2PressedLast = button2Pressed;
            }
        }.start();
    }

    /**
     * Constantly starts the given command while one button is held and the other isn't
     * Needed when using whilePressedWith() to bind a command to both buttons pressed at the same time
     *
     * {@link Command#start()} will be called repeatedly while the main button is active and 
     * the secondary button is inactive, and will be canceled when the main button becomes inactive
     *
     * @param command the command to start
     * @param button2 the second button that can't be pressed
     */
    public void whileNotPressedWith(final Command command, Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
      
            @Override
            public void execute() {
            boolean pressed = get();
            boolean button2Pressed = button2.get();

            if (pressed && !button2Pressed) {
                command.start();
            } else if (m_buttonPressedLast && !pressed && !button2Pressed) {
                command.cancel();
            }

              m_buttonPressedLast = pressed;
            }
        }.start();
    }

    /**
     * Toggles a command when two buttons become active
     *
     * @param command the command to toggle
     * @param button2 the second button that needs to be pressed
     */
    public void toggleWhenActiveWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
            private boolean m_button2PressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();

                if((pressed && !m_button2PressedLast && button2Pressed) || 
                (button2Pressed && !m_buttonPressedLast && pressed)) {
                    if(command.isRunning()) {
                        command.cancel();
                    } else {
                        command.start();
                    }
                }

                m_buttonPressedLast = pressed;
                m_button2PressedLast = button2Pressed;
            }
        }.start();
    }

    /**
     * Toggles a command when the main button is active and the secondary isn't
     * Needed when using toggleWhenActiveWith() to bind a command to both buttons pressed at the same time
     *
     * @param command the command to toggle
     * @param button2 the second button that can't be pressed
     */
    public void toggleWhenNotActiveWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();

                if(pressed && !m_buttonPressedLast && !button2Pressed) {
                    if(command.isRunning()) {
                        command.cancel();
                    } else {
                        command.start();
                    }
                }
            }
        }.start();
    }
    
    /**
     * Cancels a command when two buttons become active
     *
     * @param command the command to toggle
     * @param button2 the second button that needs to be pressed
     */
    public void cancelWhenActiveWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();
            private boolean m_button2PressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();

                if((pressed && !m_button2PressedLast && button2Pressed) || 
                (button2Pressed && !m_buttonPressedLast && pressed)) {
                    command.cancel();
                }
            }
        }.start();
    }
    
    /**
     * Cancels a command when the main button becomes active and the second button is inactive
     * Needed when using cancelWhenActiveWith() to bind a command to both buttons pressed at the same time
     *
     * @param command the command to toggle
     * @param button2 the second button that can't be pressed
     */
    public void cancelWhenNotActiveWith(final Command command, final Button button2) {
        new ButtonScheduler() {
            private boolean m_buttonPressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();
                boolean button2Pressed = button2.get();

                if(pressed && !m_buttonPressedLast && button2Pressed) {
                    command.cancel();
                }
            }
        }.start();
    }
}
