package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCMD extends CommandBase{
    private ClawSubsystem clawSubsystem;
    private boolean open;

    public ClawCMD(ClawSubsystem clawSubsystem, boolean open){
        this.clawSubsystem = clawSubsystem;
        this.open = open;

    }

@Override
public void initialize(){
    if( open ){
        clawSubsystem.setposition(0.5);
    }
}

@Override
public void execute(){
    if( !open ){
        //current based on how squishy object is
        //needs to be changed based on testing
        if( clawSubsystem.getCurrent() > 10 ){
                clawSubsystem.setposition(0.6);
        } else {
                clawSubsystem.setposition(0.4);
        }
    } else {
        clawSubsystem.setposition(0);
    }
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
