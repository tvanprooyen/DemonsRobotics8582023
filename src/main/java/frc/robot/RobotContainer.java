// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.InstantSource;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm;
import frc.robot.commands.ClawCMD;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveDrive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final ArmControl armControl = new ArmControl();
  private final ClawSubsystem claw = new ClawSubsystem();
  //private final Arm Arm = new Arm(armControl);

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
      swerveDrive.setDefaultCommand(new SwerveJoystick(
              swerveDrive,
              () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
              () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      configureButtonBindings();
  }


  private void configureButtonBindings() {
    //Swerve
    new JoystickButton(driverJoystick,2).whenPressed(() -> swerveDrive.zeroHeading());

    //Arm
    new JoystickButton(driverJoystick,4).whileTrue(new Arm(armControl, 3, 1, 0, 0));
    new JoystickButton(driverJoystick,3).whileTrue(new Arm(armControl, 0, 0.02, 0, 0));

    //Claw
    new JoystickButton(driverJoystick, 5).whileTrue(new ClawCMD(claw, true));
    new JoystickButton(driverJoystick, 6).whileTrue(new ClawCMD(claw, false));

  }

  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
       AutoConstants.kMaxSpeedMetersPerSecond,
       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 0),
        new Translation2d(1, -1)
      ),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig
    );

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveDrive::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveDrive::setModuleStates,
      swerveDrive);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveDrive.stopModules()));
  }
}