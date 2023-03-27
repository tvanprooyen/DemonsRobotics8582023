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

    private PIDController rotPID, extPID;

    private double ArmRotationSet, ExtentionSet;

    private double ArmRotationSetBuffer, ArmExtentionSetBuffer;

    private double ArmRotationError, ArmExtentionError;

    private double ExtentionSpeed;

    private boolean isExtentionEncoderReset;

    private boolean stopControl;

    private boolean AutoArmRotate;

    private boolean shouldBuffer;

    private DrivetrainSubsystem DriveTrain;

    public ArmControl(DrivetrainSubsystem DriveTrain){

        this.DriveTrain = DriveTrain;

        /* rotPID = new PIDController(0.015, 0.005, 0.00);
        extPID = new PIDController(0.1, 0, 0); */

        rotPID = new PIDController(0.015, 0.00, 0.00);
        extPID = new PIDController(0.3, 0, 0);

        //Arm Rotation Default
        this.ArmRotationSet = 160;

        this.AutoArmRotate = true;

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

        this.shouldBuffer = true;

        extMotor.setIdleMode(IdleMode.kBrake);

        //resetExtentionCount();
    }

    //----------------------------Arm Rotation----------------------------//

    /**
     * @return Rotation Encoder Posistion (Set in Spark max as 0-360 deg)
     */
    public double getRotationPosition(){
        
        return rotEncoder.getPosition();
    }

    /**
     * @param ArmRotationSet Sets the Arm Rotation PID Set Point
     */
    public void setArmRotation (double ArmRotationSet){
        this.ArmRotationSet = ArmRotationSet;
    }

    /**
     * @return Rotation PID Set Point
     */
    public double getArmRotation() {
        return this.ArmRotationSet;
    }

    /**
     * @param ArmRotationSetBuffer Sets the Arm Rotation PID Set Point Buffer (Saves point for motion to finish)
     */
    public void setArmRotationBuffer(double ArmRotationSetBuffer) {
        this.ArmRotationSetBuffer = ArmRotationSetBuffer;
    }

    /**
     * @return Rotation PID Set Point Buffer (Saves point for motion to finish)
     */
    public double getArmRotationBuffer() {
        return this.ArmRotationSetBuffer;
    }

    /**
     * @param ArmRotationSetBuffer Sets the Arm Rotation PID Error (Shows current error to setpoint, goal is 0)
     */
    public void setArmRotationError(double ArmRotationError) {
        this.ArmRotationError = ArmRotationError;
    }

    /**
     * @return Rotation PID Set Point Error (Shows current error to setpoint, goal is 0)
     */
    public double getArmRotationError() {
        return this.ArmRotationError;
    }

    /**
     * Tells us if the arm is still rotating within a tolerance
     */
    public boolean isArmRotationRunning() {
        //Acceptable Range to say its finished, There will always be an error in PID. Never fully reaches 0;
        double rotationTol = 0.03; //TODO CHANGE THIS WHEN TESTING

        return (getArmRotationError() < -rotationTol) && (getArmRotationError() > rotationTol);
    }

    public boolean isArmRotationInPosistion() {
        //Acceptable Range to say its finished, There will always be an error in PID. Never fully reaches 0;
        double rotationTol = 6; //TODO CHANGE THIS WHEN TESTING

        double armRotationSet = getArmRotation();

        if((DriveTrain.getRotationInDeg() < 45 && DriveTrain.getRotationInDeg() > -45) && getAutoArmRotate()){

            Double delta = 180 - getArmRotation();

            armRotationSet = 180 + delta;
        }


        return (getRotationPosition() > (armRotationSet - rotationTol)) && (getRotationPosition() < (armRotationSet + rotationTol));
    }

    /**
     * Runs the Arm Rotation sequence
     */
    public void runArmRotation() {
        runArmRotation(0.4);
    }

    /**
     * gets the auto rotate value
     */
    public boolean getAutoArmRotate() {
        return this.AutoArmRotate;
    }

    /**
     * Allows arm to auto rotate
     * @param AutoArmRotate true means auto rotate is active
     */
    public void setAutoArmRotate(boolean AutoArmRotate) {
        this.AutoArmRotate = AutoArmRotate;
    }

    /**
     * Runs the Arm Rotation sequence
     * @param speedLimiter Maximum speed. Keeps the PID from going to fast to reach setpoint
     */
    public void runArmRotation(double speedLimiter) {
        double ArmRotationFuturePosition = 0;

        double futureRotationSet = getArmRotation();

            if((DriveTrain.getRotationInDeg() < 45 && DriveTrain.getRotationInDeg() > -45) && getAutoArmRotate()){

                Double delta = 180 - getArmRotation();

                futureRotationSet = 180 + delta;
            }

            ArmRotationFuturePosition = rotPID.calculate(getRotationPosition(), futureRotationSet);

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

                if(ArmRotationFuturePosition > 0.30) {
                    ArmRotationFuturePosition = 0.30;
                } else if(ArmRotationFuturePosition < -0.30) {
                    ArmRotationFuturePosition = -0.30;
                }

                if(ArmRotationFuturePosition < 0) {
                    ArmRotationFuturePosition = 0;
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

    /**
     * @return Arm Extention PID Set Point
     */
    public double getArmExtention() {
        return this.ExtentionSet;
    }

    public void setArmExtentionBuffer(double ArmExtentionSetBuffer) {
        this.ArmExtentionSetBuffer = ArmExtentionSetBuffer;
    }

    /**
     * @return Arm Extention PID Set Point Buffer (Saves Set Point for Later)
     */
    public double getArmExtentionBuffer() {
        return this.ArmExtentionSetBuffer;
    }

    public void setArmExtentionError(double ArmExtentionError) {
        this.ArmExtentionError = ArmExtentionError;
    }

    /**
     * @return Extention PID Set Point Error (Shows current error to setpoint, goal is 0)
     */
    public double getArmExtentionError() {
        return this.ArmExtentionError;
    }

    /**
     * @return if arm is extended pass 1 inch (default)
     */
    public boolean isArmExtented() {
        return isArmExtented(1);
    }

    /**
     * @param extentionTol Tolerace to say arm is extended passed
     * @return if arm is extended pass the set amount
     */
    public boolean isArmExtented(double extentionTol) {
        //Get Possible Position or Actual Position
        return getArmExtention() > extentionTol || getExtentionPosition() > extentionTol;
    }

    public boolean isArmInPosistion() {
        //Get Possible Position or Actual Position
        double extentionTol = 1; //TODO CHANGE THIS WHEN TESTING

        return (getExtentionPosition() > (getArmExtention() - extentionTol)) && (getExtentionPosition() < (getArmExtention() + extentionTol));

        //return getArmExtention() > 1 || getExtentionPosition() > 1;
    }

    public boolean isArmExtentionRunning() {
        //Acceptable Range to say its finished, There will always be an error in PID. Never fully reaches 0;
        double extentionTol = 0.3; //TODO CHANGE THIS WHEN TESTING

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
        double speedLimiter = 0.75;

        ArmExtentionFuturePosition = extPID.calculate(getExtentionPosition(), getArmExtention());

        setArmExtentionError(ArmExtentionFuturePosition);

        if(!this.stopControl) {
            if(isExtentionEncoderReset == true && getArmExtentionSpeed() == -2){
                

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

            if(getRotationPosition() < 49) {
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

    public void doBuffer(boolean shouldBuffer) {
        this.shouldBuffer = shouldBuffer;
    }

    public boolean shouldBuffer() {
        return this.shouldBuffer;
    }

    public boolean setArmMotionProfile(double ArmRotationSet, double ExtentionSet, boolean interrupt) {

        if(shouldBuffer()) {
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
        } else {
            setArmExtention(ExtentionSet);
            setArmRotation(ArmRotationSet);
            setArmExtentionBuffer(-1);
            setArmRotationBuffer(-1);
        }
        

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
        
        runArmRotation(0.5);
        runArmExtention();

        if(shouldBuffer() && (!isMotionProfilerunning() && !isArmExtented())) {
            if(getArmRotationBuffer() != -1) {
                setArmRotation(getArmRotationBuffer());
                setArmRotationBuffer(-1);
            }

            if(getArmExtentionBuffer() != -1 && (isArmInPosistion() && getArmRotationBuffer() == -1)) {
                setArmExtention(getArmExtentionBuffer());
                setArmExtentionBuffer(-1);
            }
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

        SmartDashboard.putNumber("Arm Extention Voltage", Math.abs(getArmRotationError() * PD.getVoltage()));

        SmartDashboard.putNumber("Arm Rotation Voltage", Math.abs(getArmExtentionError() * PD.getVoltage()));

        SmartDashboard.putBoolean("Arm Rotating", isArmRotationRunning());

        SmartDashboard.putBoolean("Arm Extending", isArmExtentionRunning());

        SmartDashboard.putBoolean("Arm Extended", isArmExtented());

        SmartDashboard.putNumber("Arm Extention Set", getArmExtention());

        SmartDashboard.putBoolean("Arm In Posistion", isArmInPosistion());

        SmartDashboard.putBoolean("In Arm Posistion Tol", getExtentionPosition() < -(getArmExtention() - 1));

        SmartDashboard.putBoolean("Switch Forward", true);

        SmartDashboard.putBoolean("Arm Rotation in Pos", isArmRotationInPosistion());
        
    }
}