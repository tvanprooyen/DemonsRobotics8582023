package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax m_claw;
    private PIDController pid;

    public ClawSubsystem(){
        pid = new PIDController(0, 0, 0);
        m_claw = new CANSparkMax(15, MotorType.kBrushless);
    }

    public double getEncoderValue(){
        return m_claw.getEncoder().getPosition();
    }

    public void setposition(double position){
        m_claw.set(pid.calculate(  getEncoderValue(), position));
    }

    public void setSpeed(double speed){
        m_claw.set(speed);
    }

    public double getCurrent(){
        return m_claw.getOutputCurrent();
    }

}
