package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDrive;

public class SwerveJoystick extends CommandBase{

    //t = turning
    private final SwerveDrive swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, tSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, tLimiter;

    public SwerveJoystick(SwerveDrive swerveDrive,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> tSpdFunction, Supplier<Boolean> fieldOrientedFunction ){
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.tSpdFunction = tSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.tLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
    //Joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double tSpeed = tSpdFunction.get();

    //Deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        tSpeed = Math.abs(tSpeed) > OIConstants.kDeadband ? tSpeed : 0.0;

    //Smooth Driving
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        tSpeed = tLimiter.calculate(tSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    //Chassis Speed
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
        //Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, tSpeed, swerveDrive.getRotation2d());
        } else{
            //Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);
        }

        //Convert chassis Speeds to individual module states 
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output each module states to wheels
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}