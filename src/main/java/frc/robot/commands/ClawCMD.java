package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCMD extends CommandBase{
    private ClawSubsystem clawSubsystem;

    private double ClawSpeed;

    public ClawCMD(ClawSubsystem clawSubsystem, double ClawSpeed){
        this.clawSubsystem = clawSubsystem;
        this.ClawSpeed = ClawSpeed;
    }

@Override
public void initialize(){
    
}

@Override
public void execute(){
    clawSubsystem.setSpeed(this.ClawSpeed);
}

@Override
public void end(boolean interrupted) {
    clawSubsystem.setSpeed(0);
}

@Override
public boolean isFinished(){
    return false;
}

    
}
