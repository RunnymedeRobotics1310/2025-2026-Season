// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private XboxController xboxController;
  private SparkMax speedMotor;
  private SparkMax turnMotor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {
    xboxController = new XboxController(0); // Initialize XboxController on port 0
    speedMotor =
        new SparkMax(30, MotorType.kBrushless); // Initialize SparkMax on port 30 for NEO motor
    turnMotor = new SparkMax(31, MotorType.kBrushless);

    System.out.println("Im so sigma twin");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs AFTER the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    double leftY = -xboxController.getLeftY(); // Example: Get left joystick Y-axis value
    double speed = deadband(leftY);
    speedMotor.set(speed); // Set motor speed based on joystick input

    double rightX = xboxController.getRightX(); // Example: Get right joystick X-axis value
    double turn = deadband(rightX);
    turnMotor.set(turn);

    SmartDashboard.putNumber("turnMotor", rightX);
    SmartDashboard.putNumber("speedMotor", leftY);
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

  private double deadband(double value) {

    if (Math.abs(value) < 0.2) {
      return 0.0;
    }

    // FIXME: Implement a better deadband function

    return value;
  }
}
