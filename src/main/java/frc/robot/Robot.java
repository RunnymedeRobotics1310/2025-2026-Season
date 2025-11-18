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
  public Robot() {
  }

  @Override
  public void robotInit() {
    xboxController = new XboxController(0); // Initialize XboxController on port 0
    speedMotor = new SparkMax(30, MotorType.kBrushless); // Initialize SparkMax on port 30 for NEO motor
    turnMotor = new SparkMax(31, MotorType.kBrushless);
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
    double rawSpeedMotorPosition = speedMotor.getEncoder().getPosition();
    double rawSpeedMotorVelocity = speedMotor.getEncoder().getVelocity();
    double roundedSpeedMotorVelocity = Math.round(rawSpeedMotorVelocity*100.0)/100.0;
    double roundedMotorPosition = Math.round(rawSpeedMotorPosition * 100.0) / 100.0;

    double rawTurnMotorPosition = turnMotor.getEncoder().getPosition();
    double rawTurnMotorVelocity = turnMotor.getEncoder().getVelocity();
    double roundedTurnMotorVelocity = Math.round(rawTurnMotorVelocity*100.0)/100.0;
    double roundedTurnMotorPosition = Math.round(rawTurnMotorPosition * 100.0) / 100.0;
    

    SmartDashboard.putNumber("speedMotorPosition", roundedMotorPosition);
    SmartDashboard.putNumber("speedMotorVelocity", roundedSpeedMotorVelocity);
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double leftY = -xboxController.getLeftY(); // Example: Get left joystick Y-axis value
    double outY = deadBand(leftY);
    speedMotor.set(outY); // Set motor speed based on joystick input

    double rightX = xboxController.getRightX(); // Example: Get right joystick X-axis value
    double outX = deadBand(rightX);
    turnMotor.set(outX);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private double deadBand(double y) {
    if (Math.abs(y) < 0.2) {
      return (0);
    } else if (y < 0.7 && y >= 0.2) {
      return ((y - 0.2) * 0.6);
    } else if (y > -0.7 && y <= -0.2) {
      return ((y + 0.2) * 0.6);
    } else if (y <= 1.0 && y >= 0.7) {
      return ((y * (7 / 3) - (4 / 3)));
    } else if (y >= -1.0 && y <= -0.7) {
      return ((y * (7 / 3) + (4 / 3)));
    } else {
      return (0);
    }
  }

}

