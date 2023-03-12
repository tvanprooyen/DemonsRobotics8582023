package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;

public class ArmResetCommand extends CommandBase {

    private ArmControl armControl;
    
    public ArmResetCommand(ArmControl armControl) {
        this.armControl = armControl;
        addRequirements(armControl);
    }

    public ArmControl getArmControl() {
        return this.armControl;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        getArmControl().setArmExtentionSpeed(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        getArmControl().setArmExtentionSpeed(0);
        getArmControl().setArmExtentionSpeed(-2);
        getArmControl().resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
