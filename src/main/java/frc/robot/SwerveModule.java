// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class SwerveModule {

  // name of this swerve module (ie, FrontLeft)
  private final String moduleName;

  /*
   * Motors and sensors
   */
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  private CANcoder angleEncoder;

  // PID constants
  private final double cancoderOffsetDegrees, maxDriveSpeedRpm;

  /*
   * PID Constants
   */
  double Kp = .5; // Kp is the proportional gain constant

  /** SwerveModule definition for one corner of the robot */
  public SwerveModule(
      String moduleName,
      int driveMotorCanId,
      int turnMotorCanId,
      int angleEncoderCanId,
      double maxDriveSpeedRpm,
      double cancoderOffsetDegrees) {

    this.moduleName = moduleName;
    this.cancoderOffsetDegrees = cancoderOffsetDegrees;
    this.maxDriveSpeedRpm = maxDriveSpeedRpm;

    // FIXME initialize all motors and sensors
    init(driveMotorCanId, turnMotorCanId, angleEncoderCanId);
  }

  private void init(int driveMotorCanId, int turnMotorCanId, int angleEncoderCanId) {

    // FIXME initialize the motors

    // Some example intialization code
    //
    // Create a config to apply to all of the SparkMax controllers
    // SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    // sparkMaxConfig.encoder.positionConversionFactor(1.0);
    // sparkMaxConfig.encoder.velocityConversionFactor(1.0);
    // sparkMaxConfig.inverted(false);
    // sparkMaxConfig.idleMode(IdleMode.kBrake);

    // driveMotor = new SparkMax(30, MotorType.kBrushless);
    // driveMotor.configure(
    //     sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // turnMotor = new SparkMax(31, MotorType.kBrushless);
    // turnMotor.configure(
    //     sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // // Absolute encoder - used for startup position
    // angleEncoder = new CANcoder(32);

    // CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    // cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    // cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // angleEncoder.getConfigurator().apply(cancoderConfig);

  }

  public void periodic() {

    // Run the PID controllers.
    // FIXME

    // Display the current motor speed and position
    SmartDashboard.putNumber(moduleName + " Angle", round2(getAngle()));
    SmartDashboard.putNumber(moduleName + " Drive Speed", round2(getSpeed()));
    SmartDashboard.putNumber(moduleName + " Distance", round2(getDistance()));
  }

  public double getSpeed() {

    return 0; // FIXME
  }

  public void resetDistance() {
    // FIXME
  }

  public double getDistance() {

    return 0; // FIXME
  }

  public double getAngle() {

    return 0; // FIXME

    // Example Code
    //     private double encoderAngleDegrees(CANcoder angleEncoder) {
    //   double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble();

    //   double angle = rotations * 360.0;

    //   angle += CANCODER_OFFSET_DEGREES;

    //   angle = angle % 360;

    //   if (angle < 0) {
    //     angle += 360.0;
    //   }
    //   return round2(angle);
    // }
  }

  public void setSpeed() {
    // FIXME Set the pid setpoint
  }

  public void setAngle() {
    // FIXME Set the angle setpoint
  }

  /** round to two decimal places (for display) */
  private double round2(double value) {
    return Math.round(value * 100) / 100.0;
  }

  /** Speed PID controller */
  private void speedPidControl(double setPointRpm, SparkMax motor) {

    double currentSpeed = motor.getEncoder().getVelocity();
    double normalizedError = (setPointRpm - currentSpeed) / maxDriveSpeedRpm; // Normalize error
    double estimatedOutput = setPointRpm / maxDriveSpeedRpm;

    // Set the speed to estimated value trimmed by the error
    motor.set(estimatedOutput + (normalizedError * Kp));
  }

  /** Angle PID controller */
  private void anglePidControl(int setAngle, SparkMax motor) {

    double currentAngle = getAngle();
    double error = setAngle - currentAngle;
    if (Math.abs(error) > 180) {
      error -= 360 * Math.signum(error);
    }

    if (Math.abs(error) >= 3) {
      motor.set(error / 180.0);
    } else {
      motor.set(0.0);
    }
  }
}
