package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ToggleSys;
import frc.robot.subsystems.ArmControl;

//arm stuff
public class Arm extends CommandBase{

    private ArmControl armControl;
    private double ArmRotationSetCube, ExtentionSetCube, ExtentionSpeed;
    private double ArmRotationSetCone, ExtentionSetCone;
    private double ArmRotationSet, ExtentionSet;
    private boolean AutoRotate;

    private boolean disableRotation;

    private boolean isFinished;

    private ToggleSys toggleSys;

    /**
   * Sets the Arm Control Profile
   * 
   * @param armRotationControl is the arm control subsystem
   * @param AutoRotate allows the arm to rotate twords the goal
   * @param ArmRotationSet is the setpoint in deg (0-360) for the arm to go
   * @param ExtentionSet is the setpoint in inches for the extention to reach for. -1 stay at last setpoint.
   */
    public Arm (ArmControl armRotationControl, boolean AutoRotate, double ArmRotationSetCube, double ExtentionSetCube){
        this.armControl = armRotationControl;
        this.ArmRotationSetCube = ArmRotationSetCube;
        this.ExtentionSetCube = ExtentionSetCube;
        this.AutoRotate = AutoRotate;
        this.ExtentionSpeed = -2;
        disableRotation = false;
        this.isFinished = false;
        addRequirements(armRotationControl);
    }

    public Arm (ArmControl armRotationControl, boolean AutoRotate, double ArmRotationSetCube, double ExtentionSetCube, double ArmRotationSetCone, double ExtentionSetCone, ToggleSys toggleSys){
        this.armControl = armRotationControl;
        this.ArmRotationSetCube = ArmRotationSetCube;
        this.ExtentionSetCube = ExtentionSetCube;
        this.ArmRotationSetCone = ArmRotationSetCone;
        this.ExtentionSetCone = ExtentionSetCone;
        this.toggleSys = toggleSys;
        this.AutoRotate = AutoRotate;
        this.ExtentionSpeed = -2;
        disableRotation = false;
        this.isFinished = false;
        addRequirements(armRotationControl);
    }

    public Arm (ArmControl armRotationControl, double ExtentionSpeed){
        this.armControl = armRotationControl;
        this.ArmRotationSetCube = armRotationControl.getArmRotation();
        this.ExtentionSetCube = -1;
        this.AutoRotate = false;
        this.ExtentionSpeed = ExtentionSpeed;
        disableRotation = true;
        this.isFinished = false;
        addRequirements(armRotationControl);
    }

    @Override
    public void initialize(){
        if(armControl.getArmExtentionSpeed() != 0) {
            armControl.setArmExtentionSpeed(0);
            armControl.setArmExtentionSpeed(-2);
        }

        if(ExtentionSet == -1) {
            armControl.setArmExtention(armControl.getExtentionPosition());
        }

        if(toggleSys != null) {
            if(toggleSys.getToggle()) {
                this.ArmRotationSet = this.ArmRotationSetCube;
                this.ExtentionSet = this.ExtentionSetCube;
            } else {
                this.ArmRotationSet = this.ArmRotationSetCone;
                this.ExtentionSet = this.ExtentionSetCone;
            }
        } else {
            this.ArmRotationSet = this.ArmRotationSetCube;
            this.ExtentionSet = this.ExtentionSetCube;
        }
        
    }

    @Override
    public void execute(){

        if(ExtentionSpeed == -2) {
            this.isFinished = armControl.setArmMotionProfile(ArmRotationSet, ExtentionSet, false);
        } else {
            armControl.setArmExtentionSpeed(this.ExtentionSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {

        /* armControl.setArmRotation(armControl.getArmRotation());
        armControl.setArmExtention(armControl.getExtentionPosition()); */
        armControl.setArmExtentionSpeed(0);
        armControl.setArmExtentionSpeed(-2);

        this.isFinished = true;

        /* if(armControl.getExtentionPosition() < 3) {
            armControl.setArmRotation(30);
        } else {
            armControl.setArmRotation(armControl.getRotationPosition());
            armControl.setArmExtention(armControl.getExtentionPosition());
        } */
    }

    @Override
    public boolean isFinished(){
        return this.isFinished;
    }
}