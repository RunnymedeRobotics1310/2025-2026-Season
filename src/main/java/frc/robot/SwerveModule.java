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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class SwerveModule {

  /*
   * Constants
   */
  public static final double MAX_DRIVE_RPM = 6000.0; // max RPM for the drive motor

  // name of this swerve module (ie, FrontLeft)
  private final String moduleName;

  /*
   * Motors and sensors
   */
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder angleEncoder;

  private double moduleAngle;
  private double rotationAngle;

  /*
   * PID Values
   */
  private double angleSetpoint, speedSetpoint;
  private final double cancoderOffsetDegrees;
  double Kp = .5; // Kp is the proportional gain constant

  /** SwerveModule definition for one corner of the robot */
  public SwerveModule(
      String moduleName,
      int driveMotorCanId,
      int turnMotorCanId,
      int angleEncoderCanId,
      double cancoderOffsetDegrees,
      double xPosition,
      double yPosition) {

    this.moduleName = moduleName;
    this.cancoderOffsetDegrees = cancoderOffsetDegrees;

    moduleAngle = 90 - Math.toDegrees(Math.atan2(yPosition, xPosition));
    if (moduleAngle < 0) {
      moduleAngle += 360;
    }

    rotationAngle = moduleAngle + 90;
    if (rotationAngle > 360) {
      rotationAngle -= 360;
    }

    // Initialize the swerve module
    init(driveMotorCanId, turnMotorCanId, angleEncoderCanId);
  }

  private void init(int driveMotorCanId, int turnMotorCanId, int angleEncoderCanId) {

    // Create a config to apply to all of the SparkMax controllers
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.encoder.positionConversionFactor(1.0);
    sparkMaxConfig.encoder.velocityConversionFactor(1.0);
    sparkMaxConfig.inverted(false);
    sparkMaxConfig.idleMode(IdleMode.kBrake);

    driveMotor = new SparkMax(driveMotorCanId, MotorType.kBrushless);
    driveMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnMotor = new SparkMax(turnMotorCanId, MotorType.kBrushless);
    turnMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Absolute encoder - used for startup position
    angleEncoder = new CANcoder(angleEncoderCanId);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    angleEncoder.getConfigurator().apply(cancoderConfig);
  }

  public void periodic() {

    // Run the PID controllers.
    speedPidControl();
    anglePidControl();

    // Display the current motor speed and position
    SmartDashboard.putNumber(moduleName + " Angle", round2(getAngle()));
    SmartDashboard.putNumber(moduleName + " Drive Speed", round2(getSpeed()));
    SmartDashboard.putNumber(moduleName + " Distance", round2(getDistance()));
  }

  public double getSpeed() {

    return driveMotor.getEncoder().getVelocity();
  }

  public void resetDistance() {
    // FIXME
  }

  public double getDistance() {

    return driveMotor.getEncoder().getPosition();
  }

  public double getAngle() {

    double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble();

    double angle = rotations * 360.0;

    angle += cancoderOffsetDegrees;

    angle = angle % 360;

    if (angle < 0) {
      angle += 360.0;
    }

    return round2(angle);
  }

  public void setSpeed(double speed) {
    speedSetpoint = speed;
  }

  public void setAngle(double angle) {
    angleSetpoint = angle;
  }

  /** round to two decimal places (for display) */
  private double round2(double value) {
    return Math.round(value * 100) / 100.0;
  }

  /** Speed PID controller */
  private void speedPidControl() {

    double currentSpeed = driveMotor.getEncoder().getVelocity();
    double normalizedError = (speedSetpoint - currentSpeed) / MAX_DRIVE_RPM; // Normalize error
    double estimatedOutput = speedSetpoint / MAX_DRIVE_RPM;

    // Set the speed to estimated value trimmed by the error
    driveMotor.set(estimatedOutput + (normalizedError * Kp));
  }

  /** Angle PID controller */
  private void anglePidControl() {

    double currentAngle = getAngle();
    double error = angleSetpoint - currentAngle;
    if (Math.abs(error) > 180) {
      error -= 360 * Math.signum(error);
    }

    if (Math.abs(error) >= 1) {
      turnMotor.set(error / 180.0);
    } else {
      turnMotor.set(0.0);
    }
  }

  public void setRotationSpeed(double speed) {
    setAngle(rotationAngle);
    setSpeed(speed);
  }
}
