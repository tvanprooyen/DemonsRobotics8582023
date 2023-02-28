// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class SDSConstants {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    public static final int DRIVETRAIN_PIGEON_ID = 16;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(218.671);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(156.269);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(91.142);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(337.236);
    }

    //modules
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.325; //0.325
    }


    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.2;
    }
   
    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(29);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(29);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 6; //3
        public static final int kBackLeftDriveMotorPort = 3; //6
        public static final int kFrontRightDriveMotorPort = 8; //1
        public static final int kBackRightDriveMotorPort = 1; // 8

        public static final int kFrontLeftTurningMotorPort = 5; //4
        public static final int kBackLeftTurningMotorPort = 4; //5
        public static final int kFrontRightTurningMotorPort = 7; //2
        public static final int kBackRightTurningMotorPort = 2; //7

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 9;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        //dwayne testing
        /*public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 9;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 6;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 7;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 26;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 27;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 29;
        public static final int kBackRightDriveAbsoluteEncoderPort = 28;*/

        public static final int kGyroPort = 16; //TODO change when installed!

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final boolean kFrontLefTurnMotorReversed = true;
        public static final boolean kBackLeftTurnMotorReversed = true;
        public static final boolean kFrontRightTurnMotorReversed = true;
        public static final boolean kBackRightTurnMotorReversed = true;

        //public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        //public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        //public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
        
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-45); //-2.394539169967175;//130.63870544432748 /* Math.toRadians(90.5) */; /* 5.587551235365429 */;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(90);//-90 //1.527841776609421;//124.91090507193685/* Math.toRadians(0) */ /* 1.305503038093131 */;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(22.5); //2.725878350436687;//123.14850495735512/* Math.toRadians(0) */ /* -10.652893615550227 */;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(-155); //-0.332873158156872; //132.10738419070384 /* Math.toRadians(0) *//*  0 */;
        
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * Math.PI; //5
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI /* kPhysicalMaxSpeedMetersPerSecond / Math.hypot(kWheelBase / 2.0, kWheelBase / 2.0) */; /* 2 * 2 * Math.PI; */

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / Math.hypot(kWheelBase / 2.0, kWheelBase / 2.0);
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 400;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 20;
    }
    
    public static final class ArmConstants {
        public static final double kPArmRotation = 0.25;
        public static final double kPArmExtension = 0.25;

        public enum ArmMotionProfiles {
            NONE(0,0,false,false),
            LINK1(45,0.5,false,false),
            LINK2(90,11.5,false,false),
            LINK3(100,39.5,false,false);
    
    
            private double rotation,extention;
            private boolean interrupt,autorotate;
    
            ArmMotionProfiles(double rotation, double extention, boolean interrupt, boolean autorotate) {
                this.rotation = rotation;
                this.extention = extention;
                this.interrupt = interrupt;
                this.autorotate = autorotate;
            }
    
            public double getRotation() {
                return this.rotation;
            }
    
            public double getExtention() {
                return this.extention;
            }
    
            public boolean getInterrupt() {
                return this.interrupt;
            }
    
            public boolean geAutoRotate() {
                return this.autorotate;
            }
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

}

