// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private XboxController xboxController;
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder angleEncoder;

  private double DEADZONE = 0.2;
  private double SLOWZONE = 0.7;

  private double turnMotorZero;
  private double driveMototZero;

  double MAX_RPM = 6000.0; // max RPM for the motor
  double Kp = 1; // Kp is the proportional gain constant

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {
    xboxController = new XboxController(0); // Initialize XboxController on port 0
    driveMotor = new SparkMax(30, MotorType.kBrushless); // Initialize SparkMax on port 30 for NEO motor
    turnMotor = new SparkMax(31, MotorType.kBrushless);

    turnMotorZero = turnMotor.getEncoder().getPosition();

    angleEncoder = new CANcoder(32);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleEncoder.getConfigurator().apply(cancoderConfig);

    SparkMaxConfig brakeSparkConfig = new SparkMaxConfig();
    brakeSparkConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    brakeSparkConfig
        .encoder.velocityConversionFactor(1.0);
    brakeSparkConfig.encoder.positionConversionFactor(1);
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

    SmartDashboard.putNumber(
        "Angle Encoder", round2(angleEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Turn Motor Speed", round2(turnMotor.getEncoder().getVelocity()));
    SmartDashboard.putNumber("Turn Motor Position", round2(turnMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("Drive Motor Speed", round2(driveMotor.getEncoder().getVelocity()));
    SmartDashboard.putNumber("Drive Motor Position", round2(driveMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber(
        "Turn Motor Angle", angleDegrees(turnMotor));


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
  
  public double deadband(double value) {
    if (Math.abs(value) < DEADZONE) {
      return 0;
    }
    
    if(Math.abs(value) < SLOWZONE){
      return (0.6 * Math.abs(value) - 0.12) * Math.signum(value);
    } else {
      return (2.3 * Math.abs(value) - 1.3) * Math.signum(value);
    }
  }

  private void speedPidControl(double setPoint, SparkMax motor) {
    double currentSpeed = motor.getEncoder().getVelocity();
    double error = (setPoint - currentSpeed) / MAX_RPM; // Normalize error
    motor.set((setPoint / MAX_RPM) + (error * Kp));
  }
   
  private double angleDegrees(SparkMax motor) {
    // The Mk4i Swerve Module has a gear ratio of 150:7
    double angle = motor.getEncoder().getPosition() / (150.0 / 7.0) * 360.0;

    double normalized_angle = (angle % 360 + 360) % 360;

    return normalized_angle;
  }

  private double round2(double value) {
    return Math.round(value * 100) / 100.0;
  }

  private double getEncoderAngle(CANcoder encoder) {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

}
