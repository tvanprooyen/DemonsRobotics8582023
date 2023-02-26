package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final PIDController /* turningPID, */ turningSetterPID, drivePID;

  private final ProfiledPIDController turningPID;

//AbsoluteEncoder for swerve rotation, uses CTRE MAG encoder
private final CANCoder absoluteEncoder;

// Reversed, Offset
  private final boolean eabsoluteR;
  private final double AbsoluteEncoderOffsetRad;

  //Reverse Turn Motor
  private final boolean reverseTurnMotor;

  private double currentWheelPos;

public SwerveModule (
  int mdriveID, 
  int mturnID, 
  boolean mdriveReversed, 
  boolean mturnReversed,
  int eabsoluteID, 
  double AbsoluteEncoderOffsetRad, 
  boolean eabsoluteR,
  boolean reverseTurnMotor){

    this.AbsoluteEncoderOffsetRad = AbsoluteEncoderOffsetRad;
    this.eabsoluteR = eabsoluteR;

    this.reverseTurnMotor = reverseTurnMotor;

    mdrive = new CANSparkMax(mdriveID, MotorType.kBrushless);
    mturn = new CANSparkMax(mturnID, MotorType.kBrushless);

    absoluteEncoder = new CANCoder(eabsoluteID);

    
    // set units of the CANCoder to radians, with velocity being radians per second
    CANCoderConfiguration config = new CANCoderConfiguration();    
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
     config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.magnetOffsetDegrees = Math.toDegrees(AbsoluteEncoderOffsetRad);
    absoluteEncoder.configAllSettings(config);
  
    //absoluteEncoder.setPosition(0);

    mdrive.setInverted(mdriveReversed);
    mturn.setInverted(mturnReversed);

    edrive = mdrive.getEncoder();
    eturn = mturn.getEncoder();

    if(mturn.setIdleMode(IdleMode.kBrake) == REVLibError.kOk) {
      System.out.println(mturnID + " In Break Mode");
    } else {
      System.out.println(mturnID + " In Cost Mode");
    }

    edrive.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    edrive.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    eturn.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    eturn.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    //turningPID = new PIDController(0.3 /* ModuleConstants.kPTurning */, 0, 0);

    turningPID =
      new ProfiledPIDController(
          0.3,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, 2 * Math.PI));

    turningSetterPID = new PIDController(0.3, 0.00, 0.0);

    drivePID = new PIDController(2, 0, 0);

    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
 }

  public double getDrivePosition(){
    return edrive.getPosition();
  }

  public double currentDrivePosistion() {
    return this.currentWheelPos;
  }

  public void setCurrentDrivePosistion(double currentWheelPos) {
    this.currentWheelPos = currentWheelPos;
  }

  public double getTurningPosition(){
    //return getAbsoluteEncoderPosistion();
    return getAbsoluteEncoderRad();
    //return eturn.getPosition();
    //return absoluteEncoder.getPosition();
    //return getAbsoluteEncoderRad();
  }

  public double getDriveVelocity(){
    return edrive.getVelocity();
  }

  public double getTurningVelocity(){
    return eturn.getVelocity();/* absoluteEncoder.getVelocity(); */
  }

  public double getAbsoluteEncoderPosistion() {
    double angle =  absoluteEncoder.getAbsolutePosition();
    //angle *= 2.0 * Math.PI;
    return angle/*  * -1.0 */;
  }

  public double getAEPosActual() {
    return getAbsoluteEncoderPosistion() + getAbsoluteEncoderRad();
  }

  public double getAbsoluteEncoderRad(){
    
    //Get Encoder Pos
    double angle =  absoluteEncoder.getAbsolutePosition();/* absoluteEncoder.getPosition(); eturn.getPosition(); */
    //Convert to Rad
    //angle *= 2.0 * Math.PI;
    //This sets the encoder to its offset, All abs encoders have a inital posistion. Offsetting allows for is to change that.
    //angle -= AbsoluteEncoderOffsetRad;
    //In case of "Oops Wrong Way"
    return angle * (eabsoluteR ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    edrive.setPosition(0);
    eturn.setPosition(0);
    //Set to constant
    //absoluteEncoder.setPosition(0/* AbsoluteEncoderOffsetRad */);
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

  private SwerveModuleState restingState(double rads) {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(rads));
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
    mdrive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //mdrive.set(0);
    mturn.set(turningPID.calculate((reverseTurnMotor ? -1.0 : 1.0) * getAbsoluteEncoderPosistion(), AbsoluteEncoderOffsetRad + state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    
    setCurrentDrivePosistion(getDrivePosition());
  }

    public void stop() {


      SwerveModuleState state = SwerveModuleState.optimize(restingState(AbsoluteEncoderOffsetRad), getState().angle);

      mdrive.set(drivePID.calculate(getDrivePosition(), currentDrivePosistion()));
      //mturn.set(0);

      mturn.set(turningSetterPID.calculate((reverseTurnMotor ? -1.0 : 1.0) * getAbsoluteEncoderPosistion(), AbsoluteEncoderOffsetRad /* state.angle.getRadians() */));/*  + Math.toRadians(45) */

      //mturn.set(-turningPID.calculate(getTurningPosition(), Math.toRadians(45))); //Math.toRadians(45) TODO Change for norm zero, right now its at offset for testing
    }
}