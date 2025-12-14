// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  /*
   * Joystic and Deadband Calculator
   */
  private XboxController xboxController;

  private static final double DEADBAND = 0.2;
  private static final double SLOW_X = 0.7;
  private static final double SLOW_Y = .4;

  // Calculate the slope and intercept for each of the
  // slow zone and fast zone line segments.
  private static final double SLOW_M = SLOW_Y / (SLOW_X - DEADBAND);
  private static final double SLOW_B = -SLOW_M * DEADBAND;

  private static final double FAST_M = (1.0 - SLOW_Y) / (1.0 - SLOW_X);
  private static final double FAST_B = -(FAST_M * SLOW_X) + SLOW_Y;

  /*
   * Swerve Modules
   */
  private SwerveModule frontRightSwerveModule;

  /*
   * Constants
   */
  double MAX_RPM = 6000.0; // max RPM for the motor

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {

    xboxController = new XboxController(0);

    // FIXME
    frontRightSwerveModule = new SwerveModule("FrontRight", 0, 0, 0, 0, 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs AFTER the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    frontRightSwerveModule.periodic();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (xboxController.getAButton()) {
      // Set the speed to exactly 200rpm
      frontRightSwerveModule.setSpeed(200.0);
    } else if (xboxController.getBButton()) {
      speedPidControl(2000, driveMotor);
    } else if (xboxController.getYButton()) {
      speedPidControl(5000, driveMotor);
    } else {
      // Set the drive speed based on the left Y axis with deadband applied
      double leftY = -xboxController.getLeftY();
      double speed = deadband(leftY);
      frontRightSwerveModule.setSpeed(speed);
    }
    if (xboxController.getPOV() >= 0) {
      anglePidControl(xboxController.getPOV(), turnMotor);

    } else {
      // Set the turn speed based on the right X axis with deadband applied
      double rightX = xboxController.getRightX();
      double turn = deadband(rightX);
      turnMotor.set(turn);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * Deadband the input value
   *
   * @param x input value
   * @return deadbanded output value
   */
  private double deadband(double x) {

    if (Math.abs(x) < DEADBAND) {
      return 0.0;
    }
    // y = mx + b
    if (Math.abs(x) < SLOW_X) {
      return (SLOW_M * Math.abs(x) + SLOW_B) * Math.signum(x);
    }

    return (FAST_M * Math.abs(x) + FAST_B) * Math.signum(x);
  }
}
