// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final double DEADBAND = 0.2;
  private static final double SLOW_X = 0.7;
  private static final double SLOW_Y = .4;

  private static final double SLOW_M = SLOW_Y / (SLOW_X - DEADBAND);
  private static final double SLOW_B = -SLOW_M * DEADBAND;

  private static final double FAST_M = (1.0 - SLOW_Y) / (1.0 - SLOW_X);
  private static final double FAST_B = -(FAST_M * SLOW_X) + SLOW_Y;

  private XboxController xboxController;
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder angleEncoder;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);

    // Reset the speed controllers
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.encoder.positionConversionFactor(1.0);
    sparkMaxConfig.encoder.velocityConversionFactor(1.0);
    sparkMaxConfig.inverted(false);
    sparkMaxConfig.idleMode(IdleMode.kBrake);

    driveMotor = new SparkMax(30, MotorType.kBrushless);
    driveMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnMotor = new SparkMax(31, MotorType.kBrushless);
    turnMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angleEncoder = new CANcoder(32);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleEncoder.getConfigurator().apply(cancoderConfig);
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
    // Display the current motor speed and position
    SmartDashboard.putNumber(
        "Angle Encoder",
        Math.round(angleEncoder.getAbsolutePosition().getValueAsDouble() * 100) / 100.0);
    SmartDashboard.putNumber(
        "Turn Motor Speed", Math.round(turnMotor.getEncoder().getVelocity() * 100) / 100.0);
    SmartDashboard.putNumber(
        "Turn Motor Position", Math.round(turnMotor.getEncoder().getPosition() * 100) / 100.0);
    SmartDashboard.putNumber(
        "Drive Motor Speed", Math.round(driveMotor.getEncoder().getVelocity() * 100) / 100.0);
    SmartDashboard.putNumber(
        "Drive Motor Position", Math.round(driveMotor.getEncoder().getPosition() * 100) / 100.0);
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

    // Set the drive speed based on the left Y axis with deadband applied
    double leftY = -xboxController.getLeftY();
    double speed = deadband(leftY);
    driveMotor.set(speed);

    // Set the turn speed based on the right X axis with deadband applied
    double rightX = xboxController.getRightX();
    double turn = deadband(rightX);
    turnMotor.set(turn);
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

    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }

    if (Math.abs(value) < SLOW_X) {
      return (Math.abs(value) * SLOW_M + SLOW_B) * Math.signum(value);
    }

    return (Math.abs(value) * FAST_M + FAST_B) * Math.signum(value);
  }
}
