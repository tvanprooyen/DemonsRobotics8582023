package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {

    double seconds;
    Timer timer;
    boolean finished;

    public WaitCommand(double seconds) {
        this.seconds = seconds;
        this.timer = new Timer();
        this.finished = false;
    }

    @Override
    public void initialize() {
        this.timer.start();
    }

    @Override
    public void execute() {
        if(this.timer.get() > this.seconds) {
            this.finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
