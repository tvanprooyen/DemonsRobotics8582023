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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Util.MatchData;
import frc.robot.Util.ToggleSys;
import frc.robot.Util.MatchData.Actions;
import frc.robot.commands.Arm;
import frc.robot.commands.ArmResetCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LEDCMD;
import frc.robot.commands.ToggleConeCubeCommand;
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
  private final LEDControl ledControl = new LEDControl();
  private final ClawSubsystem claw = new ClawSubsystem();

  private final MatchData mData = ledControl.getMatchData();

  private final ToggleSys toggle = new ToggleSys();


  private final SlewRateLimiter xLimiter = new SlewRateLimiter(mData.getProfileSlewRate());
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(mData.getProfileSlewRate());
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(mData.getProfileSlewRate());

  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer(String driver) {
    /* ------------------------------------ IMPORTANT ------------------------------------
     * Change or Add Driver Buttons Thru MatchData.java
     * Each Driver has their own profile
     * Robot code needs to reboot inorder for changes to be in effect
     */

     mData.setDriverSelect(driver);
    
    drivetrain.register();

    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(controller.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(controller.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis((-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) + controller.getRightX())), //controller.getRightX()
            () -> controller.getPOV()
    ));

      configureButtonBindings();
  }

  public LEDControl getLedControl() {
    return ledControl;
  }


  private void configureButtonBindings() {

    boolean autoRotate = false;

    /*
      Reset Buttons
      Drive Gyro = 8/Start
      Reset Arm Encoder = 7/Back
      Low Goal/Single Station = 5/Left Bumper
      High Goal = 6/Right Bumper
      Intake = 3/X
      Release Game Object = 4/Y
      Store Arm = 1/A
    */


    /*
      Allison
      Reset Buttons
      Drive Gyro = 8/Start
      Reset Arm Encoder = 7/Back
      Low Link = 1/A
      Mid Link/Single Station = 2/B
      Top Link = 4/Y
      Release Game Object = 6/Right Bumper
      Store Arm = 5/Left Bumper
    */

    // ----------------------------------- RESETS -----------------------------------
    new JoystickButton(controller,8)
    .whileTrue(
      new InstantCommand(drivetrain::zeroGyroscope)
    );

    new JoystickButton(controller, 7)
    .whileTrue(
      new ArmResetCommand(armControl)
      //new InstantCommand(armControl::resetEncoder)
    );

    new JoystickButton(controller, 2 /* mData.getProfileButton(Actions.MODE) */)
    .onTrue(
      new ToggleConeCubeCommand(toggle)
      //new LEDCMD(1, ledControl)
    );
    
    
    // ----------------------------------- ARM CONTROL -----------------------------------
    //Mid Goal
    new JoystickButton(controller, mData.getProfileButton(Actions.MIDGOAL))
    .whileTrue(
      new ClawCMD(claw, -0.3)
    )
    .onTrue(
      new Arm(armControl, autoRotate, 
      100, 10,
      100, 10,
      toggle)
      .alongWith(
        new LEDCMD(toggle, ledControl)
      )
    );

    //High Goal
    new JoystickButton(controller,mData.getProfileButton(Actions.HIGHGOAL))
    .onTrue(
      new Arm(armControl, autoRotate, 
      115, 36,
      115, 36,
      toggle)
      .alongWith(
        new LEDCMD(toggle, ledControl)
      )
    );

    //Store
    new JoystickButton(controller,mData.getProfileButton(Actions.STORE))
    .onTrue(
      new Arm(armControl, autoRotate, 
      165, 0,
      165, 0,
      toggle)
      .alongWith(
        new LEDCMD(0, ledControl)
      )
    );

    //Claw and Intake Pose
    new JoystickButton(controller, mData.getProfileButton(Actions.LOWGOAL))
    .onTrue(
      new Arm(armControl, autoRotate, 
      50, 15,
      50, 15,
      toggle)
      .alongWith(
        new LEDCMD(toggle, ledControl)
      )
    )
    .whileTrue(
      new ClawCMD(claw, -0.3)
    );

    //Claw Out
    new JoystickButton(controller, mData.getProfileButton(Actions.RELEASE))
    .whileTrue(
      new ClawCMD(claw, 0.3)
    );

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