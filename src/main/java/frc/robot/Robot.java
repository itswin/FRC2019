/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.Lift.RocketHatchPositioningCommand;
import frc.robot.subsystems.*;
import frc.vision.DeepSpaceVisionPipeline;
import jaci.pathfinder.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  public static AHRS m_navX;
  static {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_navX = new AHRS(SerialPort.Port.kUSB); 
    } catch (RuntimeException ex ) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }
  }

  public static final PIDDriveTrain m_pidDriveTrain = new PIDDriveTrain();
  public static final Lift m_lift = new Lift();
  public static final Intake m_intake = new Intake();
  public static final HatchMechanism m_hatchMechanism = new HatchMechanism();
  public static final Paths m_paths = new Paths();
  
  // private UsbCamera camera;
  // private VisionThread visionThread;
  // private static final int kImgHeight = 480;
  // private static final int kImgWidth = 640;
  // private double centerX = 0;
  // private final Object imgLock = new Object();

  private Command m_autonomousCommand;
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    // Camera stuff
    // camera = CameraServer.getInstance().startAutomaticCapture();
    // camera.setResolution(kImgWidth, kImgHeight);
    // camera.setBrightness(75);
    // camera.setExposureManual(5);

    // visionThread = new VisionThread(camera, new DeepSpaceVisionPipeline(), pipeline -> {
    //   if (pipeline.filterContoursOutput().size() >= 2) {
    //     Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
    //     Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
    //       synchronized (imgLock) {
    //           centerX = ((r1.x + (r1.width / 2)) + (r2.x + (r2.width / 2))) / 2;
    //       }
    //   }
    // });
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println(String.format("Lift Encoders: %f.3 \t %f.3", m_lift.getLeftLiftEncoder(), m_lift.getRightLiftEncoder()));
    // System.out.println(m_lift.getLeftLimitSwitchVal() + " " + m_lift.getRightLimitSwitchVal());
    // System.out.println("Delta angle: " + m_navX.getAngle());
    // System.out.println("Setpoint a: " + m_pidDriveTrain.getSetpoint());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Robot.m_intake.retractIntake();
    Robot.m_hatchMechanism.retractHatchExtender();
    Robot.m_pidDriveTrain.stopDriveTrain();
    Robot.m_lift.setSetpoint(Robot.m_lift.kHome);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // This command needs to be started manually once
    Scheduler.getInstance().add(new RocketHatchPositioningCommand());

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    // Slow down rotation
    double rotationScalar = .75;
    // Slow down when lift is up
    double scalar = 1 - .5 * (Robot.m_lift.getRightLiftEncoder() / Robot.m_lift.getMaxRightLiftEncoderValue());

    double speed = -Robot.m_oi.driveController.getLeftYAxis() * scalar;
    double strafe = Robot.m_oi.driveController.getLeftXAxis() * scalar;
    double rotation = Robot.m_oi.driveController.getRightXAxis() * scalar * rotationScalar;
    
    // Square the values to make driving less sensitive
    // speed = speed * speed * Math.signum(speed);
    // strafe = strafe * strafe * Math.signum(strafe);
    rotation = rotation*rotation * Math.signum(rotation);

    m_pidDriveTrain.setInputSpeeds(speed, strafe, rotation);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_pidDriveTrain.setSetpoint360(m_navX.getAngle());

    // This command needs to be started manually once
    Scheduler.getInstance().add(new RocketHatchPositioningCommand());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
    // Slow down rotation
    double rotationScalar = .75;
    // Slow down when lift is up
    double scalar = 1 - .5 * (Robot.m_lift.getRightLiftEncoder() / Robot.m_lift.getMaxRightLiftEncoderValue());

    double speed = -Robot.m_oi.driveController.getLeftYAxis() * scalar;
    double strafe = Robot.m_oi.driveController.getLeftXAxis() * scalar;
    double rotation = Robot.m_oi.driveController.getRightXAxis() * scalar * rotationScalar;
    
    // Square the values to make driving less sensitive
    // speed = speed * speed * Math.signum(speed);
    // strafe = strafe * strafe * Math.signum(strafe);
    rotation = rotation*rotation * Math.signum(rotation);

    m_pidDriveTrain.setInputSpeeds(speed, strafe, rotation);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}