package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ToggleSys;

public class ToggleConeCubeCommand extends CommandBase {

    private ToggleSys toggleSys;
    private boolean finished;

    public ToggleConeCubeCommand(ToggleSys toggleSys) {
        this.toggleSys = toggleSys;
        this.finished = false;
    }

    @Override
    public void execute() {
        toggleSys.setToggle(!toggleSys.getToggle());
        this.finished = true;
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
    
}
