package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveDrive;

public class SwerveJoystick extends CommandBase{

    //t = turning
    private final SwerveDrive swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, tSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter m_xspeedLimiter, m_yspeedLimiter, m_rotLimiter;

    public SwerveJoystick(SwerveDrive swerveDrive,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> tSpdFunction, Supplier<Boolean> fieldOrientedFunction ){
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.tSpdFunction = tSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.m_xspeedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.m_yspeedLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.m_rotLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //Joystick inputs
            double xSpeedget = xSpdFunction.get();
            double ySpeedget = ySpdFunction.get();
            double tSpeedget = tSpdFunction.get();

        //Deadband
            /* xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            tSpeed = Math.abs(tSpeed) > OIConstants.kDeadband ? tSpeed : 0.0; */
        
        //Chassis Speed
        ChassisSpeeds chassisSpeeds;
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final double xSpeed =
            -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeedget, 0.3))
                * SwerveDrive.kMaxSpeed / 5;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final double ySpeed =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeedget, 0.3))
                * SwerveDrive.kMaxSpeed / 5;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final double rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(tSpeedget, 0.3))
                * SwerveDrive.kMaxAngularSpeed / 5;

                swerveDrive.drive(xSpeed, ySpeed, rot, false);
    }

    @Override
    public void end(boolean interrupted) {
        //swerveDrive.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}