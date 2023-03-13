package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ToggleSys;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.LEDControl.LEDAnimation;

public class LEDCMD extends CommandBase {

    private int indication;
    private LEDControl ledControl;
    private ToggleSys toggleSys;

    /**
     * @param indication Drived from setIndication() method -> 0 = resume, 1 = yellow, 2 = purple
     * @param ledControl Current LED Control set in Robot Container
     */
    public LEDCMD(int indication, LEDControl ledControl) {
        this.indication = indication;
        this.ledControl = ledControl;
        addRequirements(ledControl);
    }

    /**
     * @param toggleSys Allows the toggle to swich indication, this is set in the initialize() method of this class
     * @param ledControl Current LED Control set in Robot Container
     */
    public LEDCMD(ToggleSys toggleSys, LEDControl ledControl) {
        this.ledControl = ledControl;
        addRequirements(ledControl);
    }

    @Override
    public void initialize(){
        if(toggleSys != null) {
            if(toggleSys.getToggle()) {
                this.indication = 1;
            } else {
                this.indication = 2;
            }
        }
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
