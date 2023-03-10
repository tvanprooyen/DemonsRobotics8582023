package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax m_claw;
    private CANSparkMax m_claw2;

    private double ClawSpeed;

    public ClawSubsystem(){
        m_claw = new CANSparkMax(15, MotorType.kBrushless);
        m_claw2 = new CANSparkMax(17, MotorType.kBrushless);

        this.ClawSpeed = 0;
    }

    public void setSpeed(double ClawSpeed){
        this.ClawSpeed = ClawSpeed;
    }

    private double getSpeed() {
        return this.ClawSpeed;
    }

    @Override
    public void periodic() {
        m_claw.set(getSpeed());
        m_claw2.set(-getSpeed());
    }

}
