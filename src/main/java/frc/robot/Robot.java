// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  /*
   * Joystic and Deadband Calculator
   */
  private GameController gameController;

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

    gameController = new GameController(0);

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

    if (gameController.getAButton()) {
      // Set the speed to exactly 200rpm
      frontRightSwerveModule.setSpeed(200.0);
    } else if (gameController.getBButton()) {
      speedPidControl(2000, driveMotor);
    } else if (gameController.getYButton()) {
      speedPidControl(5000, driveMotor);
    } else {
      // Set the drive speed based on the left Y axis with deadband applied
      double leftY = -gameController.getLeftY();
      double speed = deadband(leftY);
      frontRightSwerveModule.setSpeed(speed);
    }
    if (gameController.getPOV() >= 0) {
      anglePidControl(gameController.getPOV(), turnMotor);

    } else {
      // Set the turn speed based on the right X axis with deadband applied
      double rightX = gameController.getRightX();
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
}
