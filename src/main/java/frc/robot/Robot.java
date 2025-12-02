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
   * Motors and sensors
   */
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder angleEncoder;

  /*
   * PID Constants
   */
  double MAX_RPM = 6000.0; // max RPM for the motor
  double Kp = .5; // Kp is the proportional gain constant

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {

    xboxController = new XboxController(0);

    // Create a config to apply to all of the SparkMax controllers
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

    // Absolute encoder - used for startup position
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
    SmartDashboard.putNumber("Angle Encoder", encoderAngleDegrees(angleEncoder, -153.4));
    SmartDashboard.putNumber("Turn Motor Speed", round2(turnMotor.getEncoder().getVelocity()));
    SmartDashboard.putNumber("Turn Motor Position", round2(turnMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("Drive Motor Speed", round2(driveMotor.getEncoder().getVelocity()));
    SmartDashboard.putNumber("Drive Motor Position", round2(driveMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("Turn Motor Angle", motorAngleDegrees(turnMotor, 16.8));
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
      speedPidControl(200, driveMotor);
    } else if (xboxController.getBButton()) {
      speedPidControl(2000, driveMotor);
    } else if (xboxController.getYButton()) {
      speedPidControl(5000, driveMotor);
    } else {
      // Set the drive speed based on the left Y axis with deadband applied
      double leftY = -xboxController.getLeftY();
      double speed = deadband(leftY);
      driveMotor.set(speed);
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

  /** round to two decimal places (for display) */
  private double round2(double value) {
    return Math.round(value * 100) / 100.0;
  }

  private void speedPidControl(double setPoint, SparkMax motor) {
    double currentSpeed = motor.getEncoder().getVelocity();
    double error = (setPoint - currentSpeed) / MAX_RPM; // Normalize error
    motor.set((setPoint / MAX_RPM) + (error * Kp));
  }

  private void anglePidControl(int setAngle, SparkMax motor) {
    double currentAngle = encoderAngleDegrees(angleEncoder, -153.4);
    double error = setAngle - currentAngle;
    if (Math.abs(error) >= 3) {
      if (error > 0) {
        motor.set(0.8);
      } else {
        motor.set(-0.8);
      }
    }
  }

  private double encoderAngleDegrees(CANcoder angleEncoder, double offsetDegrees) {
    double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble();

    double angle = rotations * 360.0;

    angle = angle % 360;

    angle += offsetDegrees;

    if (angle < 0) {
      angle += 360.0;
    }
    return round2(angle);
  }

  /** Return the current angle based on the motor encoder */
  private double motorAngleDegrees(SparkMax motor, double offset) {

    // The Mk4i Swerve Module has a gear ratio of 150:7
    double angle = motor.getEncoder().getPosition() / (150.0 / 7.0) * 360.0;

    angle += offset;

    angle = round2(angle);

    angle = angle % 360.0;

    if (angle < 0) {
      angle += 360.0;
    }

    return round2(angle);
  }
}
