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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Util.JoystickAxisAsButton;
import frc.robot.Util.MatchData;
import frc.robot.Util.ToggleSys;
import frc.robot.Util.MatchData.Actions;
import frc.robot.commands.Arm;
import frc.robot.commands.ArmResetCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LEDCMD;
import frc.robot.commands.LimeLightCommand;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.ClawCMD;
import frc.robot.subsystems.ArmControl;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.Limelight;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Limelight limelight = new Limelight();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ArmControl armControl = new ArmControl(drivetrain);
  private final LEDControl ledControl = new LEDControl();
  private final ClawSubsystem claw = new ClawSubsystem();

  private final MatchData mData = ledControl.getMatchData();

  private final ToggleSys toggle = new ToggleSys();

  

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(mData.getProfileSlewRate());
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(mData.getProfileSlewRate());
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(mData.getProfileSlewRate());

  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController controller1 = new XboxController(OIConstants.kDriverControllerPort1);

  public RobotContainer(String driver) {
    /* ------------------------------------ IMPORTANT ------------------------------------
     * Change or Add Driver Buttons Thru MatchData.java
     * Each Driver has their own profile
     * Robot code needs to reboot inorder for changes to be in effect
     */
    
    drivetrain.register();

    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            limelight,
            () -> xLimiter.calculate(modifyAxis(controller1.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(controller1.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(controller1.getRightX())), //(-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) + 
            () -> controller1.getPOV(),
            () -> controller1.getRightStickButton()
    ));

      configureButtonBindings(driver);
  }

  public LEDControl getLedControl() {
    return ledControl;
  }

  public ArmControl getArmControl() {
    return armControl;
  }


  public void configureButtonBindings(String driver) {

    mData.setDriverSelect(driver);

    boolean autoRotate = true;

    /*
      Gabe
      ----Reset Buttons----
      Drive Gyro = 8/Start
      Reset Arm Encoder = 7/Back
      ----Arm----
      Low Goal/Single Station = 5/Left Bumper
      High Goal = 6/Right Bumper
      Intake = 3/X
      Release Game Object = 4/Y
      Store Arm = 1/A
    */


    /*
      Allison
      ----Reset Buttons----
      Drive Gyro = 8/Start
      Reset Arm Encoder = 7/Back
      ----Arm----
      Low Link = 1/A
      Mid Link/Single Station = 2/B
      Top Link = 4/Y
      Release Game Object = 6/Right Bumper
      Store Arm = 5/Left Bumper
    */

    // ----------------------------------- RESETS -----------------------------------
    new JoystickButton(controller1,8)
    .whileTrue(
      new InstantCommand(drivetrain::zeroGyroscope)
    );

    new JoystickButton(controller, 7)
    .whileTrue(
      new ArmResetCommand(armControl)
      //new InstantCommand(armControl::resetEncoder)
    );

    new JoystickButton(controller, 2/* mData.getProfileButton(Actions.MODE) */)
    //.debounce(0.15)
    .onTrue(
      new SequentialCommandGroup(
      new InstantCommand(toggle::notToggle),
      new LEDCMD(toggle, ledControl)
      )
    );
    
    
    // ----------------------------------- ARM CONTROL -----------------------------------
    //Mid Goal
    new JoystickButton(controller, mData.getProfileButton(Actions.MIDGOAL))
    .onTrue(
      new SequentialCommandGroup(
        new Arm(armControl, autoRotate, 
        100, 10,
        90, 2,
        toggle,
        false),
        new WaitCommand(0.25),
        new Arm(armControl, autoRotate, 
        100, 10,
        90, 28,
        toggle,
        false),
        new WaitCommand(0.25),
        new Arm(armControl, autoRotate, 
        100, 10,
        125, 28,
        toggle,
        false)
      )

      .alongWith(
        new LEDCMD(toggle, ledControl)
      ).alongWith(
        new LimeLightCommand(limelight, toggle)
      )
    );

    //High Goal
    new JoystickButton(controller,mData.getProfileButton(Actions.HIGHGOAL))
    .onTrue(
      new SequentialCommandGroup(
       new Arm(armControl, autoRotate, 
        80, 2,
        80, 2,
        toggle,
        false),
        new WaitCommand(0.25),
        new Arm(armControl, autoRotate, 
        80, 36,
        80, 48,
        toggle,
        false),
        new WaitCommand(0.25),
        new Arm(armControl, autoRotate, 
        115, 36,
        132, 48,
        toggle,
        false)
      )
      .alongWith(
        new LEDCMD(toggle, ledControl)
      ).alongWith(
        new LimeLightCommand(limelight, toggle)
      )
    );

    //Store
    new JoystickButton(controller,mData.getProfileButton(Actions.STORE))
    .onTrue(
        new Arm(armControl, autoRotate, 
        150, 0,
        150, 0,
        toggle)
      .alongWith(
        new LEDCMD(0, ledControl)
      ).alongWith(
        new LimeLightCommand(limelight, 3)
      )
    );

    //Store inside robot
    new JoystickButton(controller,mData.getProfileButton(Actions.INSIDE)).onTrue(
      new Arm(armControl, autoRotate = false,
      40, 0,
      40, 0,
      toggle)
      .alongWith(
        new LEDCMD(0, ledControl)
      ).alongWith(
        new LimeLightCommand(limelight, 3)
      )      
    );

    //Claw and Intake Pose
    new JoystickButton(controller, mData.getProfileButton(Actions.LOWGOAL))
    .onTrue(
      new SequentialCommandGroup(
        new Arm(armControl, false, 
        90, 2,
        70, 2,
        toggle,
        false),
        new WaitCommand(0.25),
        new Arm(armControl, false, 
        50, 17,
        68.5, 2,
        toggle,
        false)
      )
      .alongWith(
        new LEDCMD(toggle, ledControl)
      ).alongWith(
        new LimeLightCommand(limelight, 3)
      )
    );

    //Claw Out
    new JoystickAxisAsButton(controller, 3)
    .whileTrue(
      new ClawCMD(claw, -0.3)
    );

    new JoystickAxisAsButton(controller, 2)
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

   public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      3,
       3)
         .setKinematics(drivetrain.kinematics)
         .setReversed(true);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(-1, 0.01)
      ),
      new Pose2d(-3, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig
    );

    PIDController xController = new PIDController(0.1, 0, 0); //TODO May need to change kP
    PIDController yController = new PIDController(0.1, 0, 0); //TODO May need to change kP
    ProfiledPIDController thetaController = new ProfiledPIDController(
      1.7, //TODO May need to change kP
      0, 
      0, 
      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      drivetrain::getPos, 
      drivetrain.kinematics, 
      xController, 
      yController, 
      thetaController, 
      drivetrain::setModuleStates,
      drivetrain);
     

     return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> drivetrain.stopModules()));  
  }
}