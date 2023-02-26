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
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontLefTurnMotorReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        DriveConstants.kFrontRightTurnMotorReversed);

        
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        DriveConstants.kBackLeftTurnMotorReversed);
     

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        DriveConstants.kBackRightTurnMotorReversed);

    //gyroscope
    private final PigeonIMU mpigeon = new PigeonIMU(DriveConstants.kGyroPort);

    //Setup Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosision(),
            frontRight.getPosision(),
            backLeft.getPosision(),
            backRight.getPosision()
        }
    );
    
    public SwerveDrive(){

        zeroHeading();

        //No Need
        /* new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start(); */
    }

    public void zeroHeading(){
        mpigeon.setFusedHeading(0.0);
    }

    public double getHeading(){
        return 0;/* Math.IEEEremainder(mpigeon.getFusedHeading(), 360); */
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    } 

     public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosision(),
                frontRight.getPosision(),
                backLeft.getPosision(),
                backRight.getPosision()
            }, 
            pose);
    } 


    @Override
    public void periodic() {
       

        odometer.update(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosision(),
                frontRight.getPosision(),
                backLeft.getPosision(),
                backRight.getPosision()
            }
        );

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("BL angle", backLeft.getTurningPosition());
        SmartDashboard.putNumber("BR angle", backRight.getTurningPosition());
        SmartDashboard.putNumber("FL angle", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FR angle", frontRight.getTurningPosition());

        SmartDashboard.putNumber("BL angle CanCoder", Math.toDegrees(backLeft.getAEPosActual()));
        SmartDashboard.putNumber("BR angle CanCoder", Math.toDegrees(backRight.getAEPosActual()));
        SmartDashboard.putNumber("FL angle CanCoder", Math.toDegrees(frontLeft.getAEPosActual()));
        SmartDashboard.putNumber("FR angle CanCoder", Math.toDegrees(frontRight.getAEPosActual()));
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
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}