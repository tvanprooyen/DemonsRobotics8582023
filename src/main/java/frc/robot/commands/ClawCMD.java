package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ToggleSys;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCMD extends CommandBase{
    private ClawSubsystem clawSubsystem;

    private double ClawSpeed;

    private ToggleSys toggleSys;

    public ClawCMD(ClawSubsystem clawSubsystem, double ClawSpeed){
        this.clawSubsystem = clawSubsystem;
        this.ClawSpeed = ClawSpeed;
        
    }

    public ClawCMD(ClawSubsystem clawSubsystem, double ClawSpeed, ToggleSys toggleSys){
        this.clawSubsystem = clawSubsystem;
        this.ClawSpeed = ClawSpeed;
        this.toggleSys = toggleSys;
        
    }

@Override
public void execute(){

    if(toggleSys != null) {
        if(!toggleSys.getToggle()) {
            this.ClawSpeed = -this.ClawSpeed;
        }
    }

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
