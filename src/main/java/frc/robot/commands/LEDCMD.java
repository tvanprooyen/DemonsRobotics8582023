package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.LEDControl.LEDAnimation;

public class LEDCMD extends CommandBase {

    private int indication;
    private LEDControl ledControl;

    /**
     * @param indication Drived from setIndication() method -> 0 = resume, 1 = yellow, 2 = purple
     * @param ledControl Current LED Control set in Robot Container
     */
    public LEDCMD(int indication, LEDControl ledControl) {
        this.indication = indication;
        this.ledControl = ledControl;
        addRequirements(ledControl);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        ledControl.setIndication(indication);
    }

    @Override
    public void end(boolean interrupted) {
        ledControl.setIndication(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
