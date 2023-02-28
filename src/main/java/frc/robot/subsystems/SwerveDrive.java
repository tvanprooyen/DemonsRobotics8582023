// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 1.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final SwerveModule m_frontLeft = new SwerveModule (
    DriveConstants.kFrontLeftDriveMotorPort, 
    DriveConstants.kFrontLeftTurningMotorPort, 
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);
  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort, 
    DriveConstants.kFrontRightTurningMotorPort, 
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);
  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort, 
    DriveConstants.kBackLeftTurningMotorPort, 
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);
  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort, 
    DriveConstants.kBackRightTurningMotorPort, 
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

  private final PigeonIMU m_gyro = new PigeonIMU(0);

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public SwerveDrive() {
    zeroHeading();
  }

  public void zeroHeading() {
    m_gyro.setFusedHeading(0);
  }

  public double getHeading() {
    return m_gyro.getFusedHeading();
  }

  public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  private void dashboard() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        /* SmartDashboard.putNumber("BL angle", backLeft.getTurningPosition());
        SmartDashboard.putNumber("BR angle", backRight.getTurningPosition());
        SmartDashboard.putNumber("FL angle", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FR angle", frontRight.getTurningPosition()); */

        /* SmartDashboard.putNumber("BL angle CanCoder", Math.toDegrees(backLeft.getAEPosActual()));
        SmartDashboard.putNumber("BR angle CanCoder", Math.toDegrees(backRight.getAEPosActual()));
        SmartDashboard.putNumber("FL angle CanCoder", Math.toDegrees(frontLeft.getAEPosActual()));
        SmartDashboard.putNumber("FR angle CanCoder", Math.toDegrees(frontRight.getAEPosActual())); */
  }

  @Override
  public void periodic() {
      updateOdometry();

      dashboard();
  }
}