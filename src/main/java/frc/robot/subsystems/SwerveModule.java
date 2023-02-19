package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;




public class SwerveModule extends SubsystemBase {

//Motors
  private final CANSparkMax mdrive;
  private final CANSparkMax mturn;

//Encoders
  private final RelativeEncoder edrive;
  private final RelativeEncoder eturn;

//PID
  private final PIDController turningPID;

//AbsoluteEncoder, Reversed, Offset
  private final AnalogInput eabsolute;
  private final boolean eabsoluteR;
  private final double AbsoluteEncoderOffsetRad;

public SwerveModule (int mdriveID, int mturnID, boolean mdriveReversed, boolean mturnReversed,
 int eabsoluteID, double AbsoluteEncoderOffsetRad, boolean eabsoluteR){

    this.AbsoluteEncoderOffsetRad = AbsoluteEncoderOffsetRad;
    this.eabsoluteR = eabsoluteR;

    mdrive = new CANSparkMax(mdriveID, MotorType.kBrushless);
    mturn = new CANSparkMax(mturnID, MotorType.kBrushless);

    eabsolute = mturn.getAnalog(Mode.kAbsolute);

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
    return eturn.getPosition();
  }

  public double getDriveVelocity(){
    return edrive.getVelocity();
  }

  public double getTurningVelocity(){
    return eturn.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = eabsolute.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= AbsoluteEncoderOffsetRad;
    return angle * (eabsoluteR ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    edrive.setPosition(0);
    eturn.setPosition(AbsoluteEncoderOffsetRad);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    
    state = SwerveModuleState.optimize(state, getState().angle);
    mdrive.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    mturn.set(turningPID.calculate(getTurningPosition(), state.angle.getRadians()));
    //SmartDashboard.putString("Swerve[" + eabsolute.getChannel() + "] state", state.toString());
  }

    public void stop() {
      mdrive.set(0);
      mturn.set(0);

    }
}