package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.LEDControl.LEDAnimation;

public class LEDCMD extends CommandBase {

    private LEDAnimation selectAnimation;
    private LEDControl ledControl;

    public LEDCMD(LEDAnimation selectAnimation, LEDControl ledControl) {
        this.selectAnimation = selectAnimation;
        this.ledControl = ledControl;
        addRequirements(ledControl);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        ledControl.selectAnimation(selectAnimation);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
