package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

  /* Size of the robot (in cm)*/
  public static final double ROBOT_LENGTH = 63;
  public static final double ROBOT_WIDTH = 58;

  /*
   * Swerve Modules
   */
  private SwerveModule rearRightSwerveModule;

  private SwerveModule frontRightSwerveModule;

  private SwerveModule rearLeftSwerveModule;

  private SwerveModule frontLeftSwerveModule;

  public SwerveSubsystem() {

    rearRightSwerveModule =
        new SwerveModule("rearRight", 30, 31, 32, -150.6, ROBOT_WIDTH, -ROBOT_LENGTH);
    frontRightSwerveModule =
        new SwerveModule("frontRight", 20, 21, 22, -349.19, ROBOT_WIDTH, ROBOT_LENGTH);
    rearLeftSwerveModule =
        new SwerveModule("rearLeft", 35, 36, 37, -25.05, -ROBOT_WIDTH, -ROBOT_LENGTH);
    frontLeftSwerveModule =
        new SwerveModule("frontLeft", 10, 11, 12, -137.4, -ROBOT_WIDTH, ROBOT_LENGTH);
  }

  @Override
  public void periodic() {
    rearRightSwerveModule.periodic();
    frontRightSwerveModule.periodic();
    rearLeftSwerveModule.periodic();
    frontLeftSwerveModule.periodic();
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

  public void setRotationSpeed(double rotationSpeed) {
    frontLeftSwerveModule.setRotationSpeed(rotationSpeed);
    rearLeftSwerveModule.setRotationSpeed(rotationSpeed);
    frontRightSwerveModule.setRotationSpeed(rotationSpeed);
    rearRightSwerveModule.setRotationSpeed(rotationSpeed);
  }
}
