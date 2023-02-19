package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmControl extends SubsystemBase {
    
    private final CANSparkMax rotMotor = new CANSparkMax(13,MotorType.kBrushless);
    private final CANSparkMax extMotor = new CANSparkMax(14,MotorType.kBrushless);

    private PIDController rotPID, extPID;

    public ArmControl(){
        rotPID = new PIDController(ArmConstants.kPArmRotation, 0, 0);
        extPID = new PIDController(ArmConstants.kPArmExtension, 0, 0);
    }

    //rotation
    public double getRotMotorValue(){
        
        return rotMotor.getAlternateEncoder(Type.kQuadrature, 1).getPosition();
    }

    public void setdesiredPositionR (double desiredPositionR){
        rotMotor.set(rotPID.calculate(getRotMotorValue(), desiredPositionR));
    }

    public void setRotSpeed (double rotSpeed){
        rotMotor.set(rotSpeed);
    }

    //extension
    public double getExtMotorValue(){
        return extMotor.getEncoder().getPosition();
    }

    public void setdesiredPositionE (double desiredPositionE){
        rotMotor.set(extPID.calculate(getRotMotorValue(), desiredPositionE));
    }

    public void setExtMotorSpeed (double extSpeed){
        rotMotor.set(extSpeed);
    }
}