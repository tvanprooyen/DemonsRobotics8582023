package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;




public class SwerveModule extends SubsystemBase {

//Spark Max Motor Controllers
  private final CANSparkMax mdrive;
  private final CANSparkMax mturn;

//Neo Encoders
  private final RelativeEncoder edrive;
  private final RelativeEncoder eturn;

//This is for rotating mturn using the Absoulte Encoder
  private final PIDController turningPID;

//AbsoluteEncoder for swerve rotation, uses CTRE MAG encoder
private final CANCoder absoluteEncoder;

// Reversed, Offset
  private final boolean eabsoluteR;
  private final double AbsoluteEncoderOffsetRad;

public SwerveModule (
  int mdriveID, 
  int mturnID, 
  boolean mdriveReversed, 
  boolean mturnReversed,
  int eabsoluteID, 
  double AbsoluteEncoderOffsetRad, 
  boolean eabsoluteR){

    this.AbsoluteEncoderOffsetRad = AbsoluteEncoderOffsetRad;
    this.eabsoluteR = eabsoluteR;

    mdrive = new CANSparkMax(mdriveID, MotorType.kBrushless);
    mturn = new CANSparkMax(mturnID, MotorType.kBrushless);

    absoluteEncoder = new CANCoder(eabsoluteID);

    /* 
    // set units of the CANCoder to radians, with velocity being radians per second
    CANCoderConfiguration config = new CANCoderConfiguration();    
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    absoluteEncoder.configAllSettings(config);
   */

    mdrive.setInverted(mdriveReversed);
    mturn.setInverted(mturnReversed);

    edrive = mdrive.getEncoder();
    eturn = mturn.getEncoder();

    edrive.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    edrive.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    eturn.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    eturn.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPID = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
 }

  public double getDrivePosition(){
    return edrive.getPosition();
  }

  public double getTurningPosition(){
    //return eturn.getPosition();
    return getAbsoluteEncoderRad();
  }

  public double getDriveVelocity(){
    return edrive.getVelocity();
  }

  public double getTurningVelocity(){
    return absoluteEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    
    //Get Encoder Pos
    double angle = absoluteEncoder.getAbsolutePosition();
    //Convert to Rad
    angle *= 2.0 * Math.PI;
    //This sets the encoder to its offset, All abs encoders have a inital posistion. Offsetting allows for is to change that.
    angle -= AbsoluteEncoderOffsetRad;
    //In case of "Oops Wrong Way"
    return angle * (eabsoluteR ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    edrive.setPosition(0);
    eturn.setPosition(0);
    //Set to constant
    absoluteEncoder.setPosition(AbsoluteEncoderOffsetRad);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosision() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    
    state = SwerveModuleState.optimize(state, getState().angle);
    mdrive.setVoltage(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    mturn.setVoltage(turningPID.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }

    public void stop() {
      mdrive.set(0);
      mturn.set(0);

    }
}