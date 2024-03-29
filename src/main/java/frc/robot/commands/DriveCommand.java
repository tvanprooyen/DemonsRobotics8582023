package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;
    private final BooleanSupplier AlignWithLimeLight;
    private final Limelight limelight;

    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            Limelight limelight,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier,
            BooleanSupplier AlignWithLimeLight
    ) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;
        this.AlignWithLimeLight = AlignWithLimeLight;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        boolean startLimeLightAlign = AlignWithLimeLight.getAsBoolean();
        int povPos = povSupplier.getAsInt();

        //Allows the Robot to angle its self if the POV is pressed and with in the deadband of the axis
        if(rotationPercent < 0.05 && rotationPercent > -0.05) {
            if(povPos != -1) {
                //-360 flips the axis
                drivetrain.setPIDRotateValue(360 - povPos);
                drivetrain.setRotateLock(true);
            }
        } else {
            drivetrain.setRotateLock(false);
        }

        if(drivetrain.getRotateLock()) {
            rotationPercent = -drivetrain.rotatePIDCalculation();

            if(rotationPercent > 0.2) {
                rotationPercent = 0.2;
            } else if(rotationPercent < -0.2) {
                rotationPercent = -0.2;
            }
        }

        //LimeLight
        if(startLimeLightAlign) {
            if(drivetrain.getRotationInDeg() < 90 && drivetrain.getRotationInDeg() > -90) {
                translationYPercent -= limelight.PIDControllerCalculation();
            } else {
                translationYPercent += limelight.PIDControllerCalculation();
            }
        }

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

}