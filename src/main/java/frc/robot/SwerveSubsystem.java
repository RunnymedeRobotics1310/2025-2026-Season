package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NavX.AHRS;

public class SwerveSubsystem extends SubsystemBase {

  /*
   * Gyro
   */
  private AHRS navX = new AHRS();

  /*
   * Swerve Modules
   */
  private SwerveModule rearRightSwerveModule;

  private SwerveModule frontRightSwerveModule;

  private SwerveModule rearLeftSwerveModule;

  private SwerveModule frontLeftSwerveModule;

  public SwerveSubsystem() {

    rearRightSwerveModule = new SwerveModule("rearRight", 30, 31, 32, -150.6);
    frontRightSwerveModule = new SwerveModule("frontRight", 20, 21, 22, -349.19);
    rearLeftSwerveModule = new SwerveModule("rearLeft", 35, 36, 37, -25.05);
    frontLeftSwerveModule = new SwerveModule("frontLeft", 10, 11, 12, -137.4);
  }

  @Override
  public void periodic() {
    rearRightSwerveModule.periodic();
    frontRightSwerveModule.periodic();
    rearLeftSwerveModule.periodic();
    frontLeftSwerveModule.periodic();

    SmartDashboard.putData("NavX", navX);
  }

  public void setSpeed(double speed) {
    frontLeftSwerveModule.setSpeed(speed);
    rearLeftSwerveModule.setSpeed(speed);
    frontRightSwerveModule.setSpeed(speed);
    rearRightSwerveModule.setSpeed(speed);
  }

  public void setAngle(double angle) {
    frontLeftSwerveModule.setAngle(angle);
    rearLeftSwerveModule.setAngle(angle);
    frontRightSwerveModule.setAngle(angle);
    rearRightSwerveModule.setAngle(angle);
  }
}
