// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SDSConstants;
import frc.robot.commands.Arm;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LEDCMD;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.ClawCMD;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
//import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.LEDControl;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmControl armControl = new ArmControl();
  private final LEDControl ledControl1 = new LEDControl();
  private final ClawSubsystem claw = new ClawSubsystem();


  private final SlewRateLimiter xLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(15);
  

  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    drivetrain.register();

    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(controller.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(controller.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(controller.getRightX()))
    ));

      configureButtonBindings();
  }

  public LEDControl getLedControl() {
    return ledControl1;
  }


  private void configureButtonBindings() {
    //Swerve
    //new JoystickButton(controller,2).whileTrue(Commands.runOnce(() -> drivetrain.zeroGyroscope()));
    
    new JoystickButton(controller,2).whileTrue(new InstantCommand(drivetrain::zeroGyroscope));

    //Arm
   /* new JoystickButton(controller,3).onTrue(new Arm(armControl, false, 90, 0.5));
    new JoystickButton(controller,4).onTrue(new Arm(armControl, false, 270, 0.5));

    new JoystickButton(controller,7).onTrue(new Arm(armControl, false, 40, 0.5));*/

    new JoystickButton(controller,5).onTrue(new Arm(armControl, false, 100, 10));

    new JoystickButton(controller,1).onTrue(new Arm(armControl, false, 50, 0));

    new JoystickButton(controller,6).onTrue(new Arm(armControl, false, 115, 36));

    new JoystickButton(controller, 7).onTrue(new Arm(armControl, false, 190, 0));

    new JoystickButton(controller, 8).whileTrue(new InstantCommand(armControl::resetEncoder));

    //Further Pole 39.5 | Closer Pole 11.5
    /*new JoystickButton(controller,5).onTrue( new Arm(armControl, false, 110, 20 /* 11.5 ));
    new JoystickButton(controller,6).onTrue(new Arm(armControl, false, 115, 39 /* 39.5 ));

    new JoystickButton(controller,1).onTrue(new Arm(armControl, -0.3));
    new JoystickButton(controller,2).onTrue(new Arm(armControl, 0.3));
    
    new JoystickButton(controller,8).whileTrue(Commands.runOnce(() -> armControl.resetEncoder()));
    //new JoystickButton(controller,3).whileTrue(new Arm(armControl, 0, 0.02, 0, 0));
    */
    //Claw
    new JoystickButton(controller, 3).whileTrue(new ClawCMD(claw, 0.3));
    new JoystickButton(controller, 4).whileTrue(new ClawCMD(claw, - 0.3));

  }

  public DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
}

private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
}

private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
}
/* 
   public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         .setKinematics(drivetrain.kinematics);

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
    thetaController.enableContinuousInput(-Math.PI, Math.PI); */

    

    //SwerveControllerCommand s = new SwerveControllerCommand(trajectory, drivetrain::robotPose, drivetrain.kinematics, xController, yController, thetaController, null, drivetrain);
    /* 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      (drivetrain::getPos),
      drivetrain.kinematics,
      xController,
      yController,
      thetaController,
      drivetrain::setModuleStates,
      drivetrain);
      */

    /* return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveDrive.stopModules()));  
  }*/
}