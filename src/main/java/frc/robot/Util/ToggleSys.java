package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToggleSys extends SubsystemBase {

    private boolean toggle;

    public ToggleSys() {
        setToggle(true);
    }

    public boolean getToggle() {
        return this.toggle;
    }

    public void setToggle(boolean toggle) {
        this.toggle = toggle;
        SmartDashboard.putBoolean("Toggle Sys", this.toggle);
    }

    public void notToggle() {
        setToggle(!getToggle());
    }
    
}
