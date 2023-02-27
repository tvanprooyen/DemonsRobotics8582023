package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmControl extends SubsystemBase {
    
    private final CANSparkMax rotMotor = new CANSparkMax(13,MotorType.kBrushless);
    private final CANSparkMax extMotor = new CANSparkMax(14,MotorType.kBrushless);

    private final PowerDistribution PD = new PowerDistribution(20, ModuleType.kRev);

    private final AbsoluteEncoder rotEncoder, extEncoder;

    private final Timer timer = new Timer();

    private PIDController rotPID, extPID;

    private double ArmRotationSet, ExtentionSet;

    private double ArmRotationSetBuffer, ArmExtentionSetBuffer;

    private double ArmRotationError, ArmExtentionError;

    private double ExtentionSpeed;

    private boolean isExtentionEncoderReset;

    private boolean stopControl;

    public ArmControl(){
        rotPID = new PIDController(0.015, 0.005, 0.00);
        extPID = new PIDController(0.1, 0, 0);

        //Arm Rotation Default
        this.ArmRotationSet = 50;

        this.ExtentionSpeed = 0;

        rotEncoder = rotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        extEncoder = extMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        this.ExtentionSet = getExtentionPosition();

        this.isExtentionEncoderReset = false;

        this.stopControl = false;

        this.ArmRotationSetBuffer = -1;

        this.ArmExtentionSetBuffer = -1;

        this.ArmRotationError = 0;
        this.ArmExtentionError = 0;

        this.timer.reset();

        extMotor.setIdleMode(IdleMode.kBrake);

        //resetExtentionCount();
    }

    //----------------------------Arm Rotation----------------------------//

    //rotation
    public double getRotationPosition(){
        
        return rotEncoder.getPosition();
    }

    public void setArmRotation (double ArmRotationSet){
        this.ArmRotationSet = ArmRotationSet;
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

    public void setArmRotationError(double ArmRotationError) {
        this.ArmRotationError = ArmRotationError;
    }

    public double getArmRotationError() {
        return this.ArmRotationError;
    }

    public boolean isArmRotationRunning() {
        //Acceptable Range to say its finished, There will always be an error in PID. Never fully reaches 0;
        double rotationTol = 0.03; //TODO CHANGE THIS WHEN TESTING

        return (getArmRotationError() < -rotationTol) && (getArmRotationError() > rotationTol);
    }

    public void runArmRotation() {
        double ArmRotationFuturePosition = 0;
        double speedLimiter = 0.3;

        if(!this.stopControl) {

            ArmRotationFuturePosition = rotPID.calculate(getRotationPosition(), getArmRotation());

            setArmRotationError(ArmRotationFuturePosition);

            if(ArmRotationFuturePosition > speedLimiter) {
                ArmRotationFuturePosition = speedLimiter;
            } else if(ArmRotationFuturePosition < -speedLimiter) {
                ArmRotationFuturePosition = -speedLimiter;
            }

            if(getRotationPosition() < 30) {
                if(ArmRotationFuturePosition > (speedLimiter - 0.3)) {
                    ArmRotationFuturePosition = (speedLimiter - 0.3);
                } else if(ArmRotationFuturePosition < -(speedLimiter - 0.3)) {
                    ArmRotationFuturePosition = -(speedLimiter - 0.3);
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
        this.ExtentionSet = ExtentionSet;
    }

    public double getArmExtention() {
        return this.ExtentionSet;
    }

    public void setArmExtentionBuffer(double ArmExtentionSetBuffer) {
        this.ArmExtentionSetBuffer = ArmExtentionSetBuffer;
    }

    public double getArmExtentionBuffer() {
        return this.ArmExtentionSetBuffer;
    }

    public void setArmExtentionError(double ArmExtentionError) {
        this.ArmExtentionError = ArmExtentionError;
    }

    public double getArmExtentionError() {
        return this.ArmExtentionError;
    }

    public boolean isArmExtented() {
        //Get Possible Position or Actual Position
        return getArmExtention() > 1 || getExtentionPosition() > 1;
    }

    public boolean isArmExtentionRunning() {
        //Acceptable Range to say its finished, There will always be an error in PID. Never fully reaches 0;
        double extentionTol = 0.03; //TODO CHANGE THIS WHEN TESTING

        return (getArmExtentionError() < -extentionTol) && (getArmExtentionError() > extentionTol);
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
        this.isExtentionEncoderReset = true;
    }

    public void runArmExtention() {
        double ArmExtentionFuturePosition = 0;
        double speedLimiter = 0.65;

        if(!this.stopControl) {
            if(isExtentionEncoderReset == true && getArmExtentionSpeed() == -2){
                
                ArmExtentionFuturePosition = extPID.calculate(getExtentionPosition(), getArmExtention());

                setArmExtentionError(ArmExtentionFuturePosition);

                if(ArmExtentionFuturePosition > speedLimiter) {
                    ArmExtentionFuturePosition = speedLimiter;
                } else if(ArmExtentionFuturePosition < -speedLimiter) {
                    ArmExtentionFuturePosition = -speedLimiter;
                }

            } else {
                if(getArmExtentionSpeed() != -2) {
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
            
        }

        extMotor.set(ArmExtentionFuturePosition);
    }

    //----------------------------Other----------------------------//

    public boolean setArmMotionProfile(double ArmRotationSet, double ExtentionSet, boolean interrupt) {

        if((ArmRotationSet != getArmRotation() && isArmExtented())) {
            setArmRotationBuffer(ArmRotationSet);
            setArmExtentionBuffer(ExtentionSet);
            setArmExtention(0.5);
            return true;
        } else {
            if(!isArmExtentionRunning()) {
                if(getArmRotationBuffer() != -1) {
                    setArmRotation(getArmRotationBuffer());
                    setArmRotationBuffer(-1);
                } else {
                    if(getArmRotation() != ArmRotationSet) {
                        setArmRotation(ArmRotationSet);
                        setArmRotationBuffer(-1);
                    }
                } 
            }
        }

        if(isArmExtentionRunning()) {
            if(getArmExtentionBuffer() == -1) {
                setArmExtentionBuffer(ExtentionSet);
            }
            
        } else {
            if(!isArmRotationRunning()) {
                if(getArmExtentionBuffer() != -1) {
                    setArmExtention(getArmExtentionBuffer());
                    setArmExtentionBuffer(-1);
                } else {
                    if(getArmExtention() != ExtentionSet) {
                        setArmExtention(ExtentionSet);
                        setArmExtentionBuffer(-1);
                    }
                }
            }
            
        }


        this.timer.reset();
        this.timer.start();
        return true;
        
    }

    public boolean isMotionProfilerunning() {
        return isArmExtentionRunning() || isArmRotationRunning();
    }

    public void stop() {
        this.stopControl = true;
    }

    public void resetStop() {
        this.stopControl = false;
    }

    @Override
    public void periodic() {

        runArmRotation();
        runArmExtention();

        if(!isMotionProfilerunning() && !isArmExtented()/*  && this.timer.get() > 1 */) {
            if(getArmRotationBuffer() != -1) {
                setArmRotation(getArmRotationBuffer());
                setArmRotationBuffer(-1);
            }

            if(getArmExtentionBuffer() != -1) {
                setArmExtention(getArmExtentionBuffer());
                setArmExtentionBuffer(-1);
            }

            this.timer.stop();
        }


        dashbard();
    }

    private void dashbard(){
        SmartDashboard.putNumber("Arm Extention", getExtentionPosition());
        SmartDashboard.putNumber("Arm Encoder", extMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("Arm Rotation", getRotationPosition());

        SmartDashboard.putNumber("Arm Rotation Set", getArmRotation());

        SmartDashboard.putNumber("Arm Rotation Set Buffer", getArmRotationBuffer());

        SmartDashboard.putNumber("Arm Extention Set Buffer", getArmExtentionBuffer());

        SmartDashboard.putNumber("Arm Rotation Error", getArmRotationError());

        SmartDashboard.putNumber("Arm Extention Error", getArmExtentionError());

        SmartDashboard.putNumber("Arm Extention Voltage", getArmRotationError() * PD.getVoltage());

        SmartDashboard.putNumber("Arm Rotation Voltage", getArmExtentionError() * PD.getVoltage());

        SmartDashboard.putBoolean("Arm Rotating", isArmRotationRunning());

        SmartDashboard.putBoolean("Arm Extending", isArmExtentionRunning());

        SmartDashboard.putBoolean("Arm Extended", isArmExtented());
        
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