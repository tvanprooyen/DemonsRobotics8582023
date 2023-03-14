package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ToggleSys;
import frc.robot.subsystems.Limelight;

public class LimeLightCommand extends CommandBase {
    
    private Limelight lmlt;

    private ToggleSys toggle;

    private int pipeline;

    public LimeLightCommand(Limelight lmlt, int pipeline) {

        this.lmlt = lmlt;
        this.pipeline = pipeline;

        addRequirements(lmlt);
    }

    public LimeLightCommand(Limelight lmlt, ToggleSys toggle) {

        this.lmlt = lmlt;
        this.pipeline = 0;
        this.toggle = toggle;

        addRequirements(lmlt);
    }

    @Override
    public void execute() {
        if(toggle != null) {
            if(toggle.getToggle()) {
                pipeline = 1;
            } else {
                pipeline = 0;
            }
        }
        
        lmlt.setPipeLine(pipeline);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
