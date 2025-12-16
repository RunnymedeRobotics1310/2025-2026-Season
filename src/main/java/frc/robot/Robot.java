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
  private SwerveModule rearRightSwerveModule;

  private SwerveModule frontRightSwerveModule;

  private SwerveModule rearLeftSwerveModule;

  private SwerveModule frontLeftSwerveModule;

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

    rearRightSwerveModule = new SwerveModule("rearRight", 30, 31, 32, MAX_RPM, -150.6);
    frontRightSwerveModule = new SwerveModule("frontRight", 20, 21, 22, MAX_RPM, -349.19);
    rearLeftSwerveModule = new SwerveModule("rearLeft", 35, 36, 37, MAX_RPM, -25.05);
    frontLeftSwerveModule = new SwerveModule("frontLeft", 10, 11, 12, MAX_RPM, -137.4);
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

    rearRightSwerveModule.periodic();
    frontRightSwerveModule.periodic();
    rearLeftSwerveModule.periodic();
    frontLeftSwerveModule.periodic();
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
      setSpeed(200.0);
    } else if (gameController.getBButton()) {
      setSpeed(2000.0);
    } else if (gameController.getYButton()) {
      setSpeed(5000.0);

    } else {
      // Set the drive speed based on the left Y axis with deadband applied
      double speed = gameController.getLeftY();
      setSpeed(MAX_RPM * speed);
    }
    if (gameController.getPOV() >= 0) {
      setAngle(gameController.getPOV());

    } else {
      // Set the turn angle based on the angle of the right joystick
      double rightX = gameController.getRightX();
      double rightY = gameController.getRightY();
      double angleRad = Math.atan2(rightY, rightX);
      double angleDeg = Math.toDegrees(angleRad) - 90;
      if (angleDeg < 0) angleDeg += 360;
      if (rightX != 0 || rightY != 0) setAngle(angleDeg);
    }
  }

  private void setSpeed(double speed) {
    frontLeftSwerveModule.setSpeed(speed);
    rearLeftSwerveModule.setSpeed(speed);
    frontRightSwerveModule.setSpeed(speed);
    rearRightSwerveModule.setSpeed(speed);
  }

  private void setAngle(double angle) {
    frontLeftSwerveModule.setAngle(angle);
    rearLeftSwerveModule.setAngle(angle);
    frontRightSwerveModule.setAngle(angle);
    rearRightSwerveModule.setAngle(angle);
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
