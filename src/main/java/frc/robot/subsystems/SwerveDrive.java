package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ModuleConstants;
//import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    //gyroscope
    private final PigeonIMU mpigeon = new PigeonIMU(0);

    
    private final SwerveModulePosition frontLeftPosistion = new SwerveModulePosition(frontLeft.getState().speedMetersPerSecond, frontLeft.getState().angle);
    private final SwerveModulePosition backLeftPosistion = new SwerveModulePosition(backLeft.getState().speedMetersPerSecond, backLeft.getState().angle);
    private final SwerveModulePosition frontRightPosistion = new SwerveModulePosition(frontRight.getState().speedMetersPerSecond, frontRight.getState().angle);
    private final SwerveModulePosition backRightPosistion = new SwerveModulePosition(backRight.getState().speedMetersPerSecond, backRight.getState().angle);

    private final SwerveModulePosition[] SMP = {frontLeftPosistion, backLeftPosistion, frontRightPosistion, backRightPosistion};


    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0), SMP);
    
    public SwerveDrive(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    public void zeroHeading(){
        mpigeon.setYaw(0);
    }

    public double getHeading(){
        return Math.IEEEremainder(mpigeon.getAbsoluteCompassHeading(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    } 

     public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), SMP, pose);
    } 


    @Override
    public void periodic() {
       

        odometer.update(getRotation2d(), SMP);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("BL angle", backLeft.getTurningPosition()*2*Math.PI);
        SmartDashboard.putNumber("BR angle", backRight.getTurningPosition()*2*Math.PI);
        SmartDashboard.putNumber("FL angle", frontLeft.getTurningPosition()*2*Math.PI);
        SmartDashboard.putNumber("FR angle", frontRight.getTurningPosition()*2*Math.PI);
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        frontRight.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}