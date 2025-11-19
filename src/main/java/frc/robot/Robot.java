// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private SparkMax speedMotor;
  private SparkMax turnMotor;

  private double DEADZONE = 0.2;
  private double SLOWZONE = 0.7;

  private double turnMotorZero;
  private double driveMototZero;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {
    xboxController = new XboxController(0); // Initialize XboxController on port 0
    speedMotor = new SparkMax(30, MotorType.kBrushless); // Initialize SparkMax on port 30 for NEO motor
    turnMotor = new SparkMax(31, MotorType.kBrushless);

    turnMotorZero = turnMotor.getEncoder().getPosition();


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

    SmartDashboard.putNumber("Trun speed", turnMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Turn angle", (turnMotor.getEncoder().getPosition() - turnMotorZero) * 360);


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
    double leftY = -xboxController.getLeftY();
    double rightX = xboxController.getRightX();

    if (rightX <= 0.05 && rightX >= -0.05) {
      rightX = 0;
    }

    speedMotor.set(deadband(leftY));
    turnMotor.set(deadband(rightX));

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
}
