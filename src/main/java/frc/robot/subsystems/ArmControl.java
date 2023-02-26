package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmControl extends SubsystemBase {
    
    private final CANSparkMax rotMotor = new CANSparkMax(13,MotorType.kBrushless);
    private final CANSparkMax extMotor = new CANSparkMax(14,MotorType.kBrushless);

    private final PowerDistribution PD = new PowerDistribution(20, ModuleType.kRev);

    private final AbsoluteEncoder rotEncoder, extEncoder;


    private PIDController rotPID, extPID;

    private double ArmRotationSet, ExtentionSet;

    private double ArmRotationSetBuffer;

    private double ExtentionSpeed;

    private boolean isExtentionEncoderReset;

    private boolean stopControl;

    public ArmControl(){
        rotPID = new PIDController(0.020, 0.001, 0);
        extPID = new PIDController(0.1, 0, 0);

        //Arm Rotation Default
        this.ArmRotationSet = 45;

        this.ExtentionSpeed = 0;

        rotEncoder = rotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        extEncoder = extMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        this.ExtentionSet = getExtentionPosition();

        this.isExtentionEncoderReset = false;

        this.stopControl = false;

        this.ArmRotationSetBuffer = -1;

        extMotor.setIdleMode(IdleMode.kBrake);

        //resetExtentionCount();
    }

    //----------------------------Arm Rotation----------------------------//

    //rotation
    public double getRotationPosition(){
        
        return rotEncoder.getPosition();
    }

    public void setArmRotation (double ArmRotationSet){
        if(!(getExtentionPosition() > 1 || getArmExtention() > 1)) {
            this.ArmRotationSet = ArmRotationSet;
            this.ArmRotationSetBuffer = -1;
        } else {
            this.ArmRotationSetBuffer = this.ArmRotationSet;
        }
        
    }

    public double getArmRotation() {
        return this.ArmRotationSet;
    }

    public void setArmRotationBuffer(double ArmRotationSetBuffer) {
        this.ArmRotationSetBuffer = ArmRotationSetBuffer;
    }

    public double getArmRotationBuffer() {
        return this.ArmRotationSetBuffer;
    }

    public void runArmRotation() {
        double ArmRotationFuturePosition = 0;

        if(!this.stopControl) {

            if(this.ArmRotationSetBuffer !=-1 && (getArmExtention() < 1)) {
                this.ArmRotationSet = this.ArmRotationSetBuffer;
                this.ArmRotationSetBuffer = -1;
            }

                ArmRotationFuturePosition = rotPID.calculate(getRotationPosition(), ArmRotationSet);

            if(ArmRotationFuturePosition > 0.40) {
                ArmRotationFuturePosition = 0.40;
            } else if(ArmRotationFuturePosition < -0.40) {
                ArmRotationFuturePosition = -0.40;
            }

            if(getRotationPosition() < 30) {
                if(ArmRotationFuturePosition > 0.10) {
                    ArmRotationFuturePosition = 0.10;
                } else if(ArmRotationFuturePosition < -0.10) {
                    ArmRotationFuturePosition = -0.10;
                }
            }

            //DO NOT EDIT! THIS WILL MAKE THE MOTOR BREAK
            if(getRotationPosition() > 340 && getRotationPosition() < 360) {
                if(ArmRotationFuturePosition > 0.05) {
                    ArmRotationFuturePosition = 0.05;
                } else if(ArmRotationFuturePosition < 0) {
                    ArmRotationFuturePosition = 0;
                }
            }

            if(getRotationPosition() < 60 && (getExtentionPosition() > 1 || getArmExtention() > 1)) {

                if(ArmRotationFuturePosition > 0.10) {
                    ArmRotationFuturePosition = 0.10;
                } else if(ArmRotationFuturePosition < -0.10) {
                    ArmRotationFuturePosition = -0.10;
                }

                if(ArmRotationFuturePosition < 0) {
                    ArmRotationFuturePosition = 0;
                }
            }
        }

        rotMotor.set(ArmRotationFuturePosition);

        
        SmartDashboard.putNumber("Arm Speed", ArmRotationFuturePosition * PD.getVoltage());
    }

    //----------------------------Arm Extention----------------------------//

    /**
     * @return the posistion in inches
     */
    public double getExtentionPosition(){
        
        //6.9092 is the nearest converstion factor
        return extMotor.getEncoder().getPosition()*6.9092;
    }

    /**
     * @param ExtentionSet set the posistion in inches
     */
    public void setArmExtention (double ExtentionSet){
        if(ExtentionSet == -1) {
            this.ExtentionSet = getExtentionPosition();
        } else {
            this.ExtentionSet = ExtentionSet;
        }
    }

    public double getArmExtention() {
        return this.ExtentionSet;
    }

    public void setArmExtentionSpeed(double ExtentionSpeed) {
        this.ExtentionSpeed = ExtentionSpeed;
    }

    public double getArmExtentionSpeed() {
        return this.ExtentionSpeed;
    }

    /**
     * resets the extention encoder, sets motor mode(break), and allows PID to start
     */
    public void resetEncoder() {
        setArmExtention(0);
        extMotor.getEncoder().setPosition(0);
        //extMotor.setIdleMode(IdleMode.kBrake);
        this.isExtentionEncoderReset = true;
        //this.stopControl = false;
    }

    public void runArmExtention() {
        double ArmExtentionFuturePosition = 0;
        double speedLimiter = 0.6;

        if(!this.stopControl) {
            if(isExtentionEncoderReset == true && this.ExtentionSpeed == -2){

                if((/* getArmRotation() !=  */this.ArmRotationSetBuffer == -1)) {
                    ArmExtentionFuturePosition = extPID.calculate(getExtentionPosition(), 0.5);
                } else {
                    ArmExtentionFuturePosition = extPID.calculate(getExtentionPosition(), this.ExtentionSet);
                }

                if(ArmExtentionFuturePosition > speedLimiter) {
                    ArmExtentionFuturePosition = speedLimiter;
                } else if(ArmExtentionFuturePosition < -speedLimiter) {
                    ArmExtentionFuturePosition = -speedLimiter;
                }

            } else {
                if(this.ExtentionSpeed != -2) {
                    setArmExtention(getExtentionPosition());
                    ArmExtentionFuturePosition = this.ExtentionSpeed;
                }
            }

            if(getRotationPosition() < 60) {
                if(ArmExtentionFuturePosition > 0) {
                    ArmExtentionFuturePosition = 0;
                }
            }
    
            //Soft limits
            /* if((getExtentionPosition() < 1 && ArmExtentionFuturePosition < 0 ) || getExtentionPosition() > 45 && ArmExtentionFuturePosition > 0) {
                ArmExtentionFuturePosition = 0;
            } */
            
            SmartDashboard.putNumber("Arm Extention Speed", ArmExtentionFuturePosition * PD.getVoltage());
        }

        extMotor.set(ArmExtentionFuturePosition);
    }

    //----------------------------Other----------------------------//

    public void stop() {
        this.stopControl = true;
    }

    public void resetStop() {
        this.stopControl = false;
    }

    @Override
    public void periodic() {

        /* runArmRotation();
        runArmExtention(); */

        dashbard();
    }

    private void dashbard(){
        SmartDashboard.putNumber("Arm Extention", getExtentionPosition());
        SmartDashboard.putNumber("Arm Encoder", extMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("Arm Rotation", getRotationPosition());

        SmartDashboard.putNumber("Arm Rotation Set", getArmRotation());

        SmartDashboard.putNumber("Arm Rotation Set Buffer", this.ArmRotationSetBuffer);
    }
}





/* public void runExtentionSpeed() {
    double ArmExtentionFuturePosition;

if(ExtentionSet == 0) {
    ArmExtentionFuturePosition = extPID.calculate(getExtentionPosition(), ExtentionSet);

    extMotor.set(ArmExtentionFuturePosition);
} else {
    ExtentionSet = getExtentionPosition();
}

extMotor.set(this.ExtentionSpeed);
} */