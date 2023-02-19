package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import edu.wpi.first.math.controller.PIDController;

//arm stuff
public class Arm extends CommandBase{

    private ArmControl armControl;
    private double desiredPositionR, rotSpeed, desiredPositionE, extSpeed;
    

public Arm (ArmControl armRotationControl, double desiredPositionR, double rotSpeed, double desiredPostionE, double extSpeed){
    this.armControl = armRotationControl;
    this.desiredPositionR = desiredPositionR;
    this.rotSpeed = rotSpeed;
    this.desiredPositionE = desiredPostionE;
    this.extSpeed = extSpeed;
    addRequirements(armRotationControl);
}    

@Override
public void initialize(){

}

@Override
public void execute(){
    if( desiredPositionR != 0){
        armControl.setdesiredPositionR(desiredPositionR);
    } else {
        armControl.setRotSpeed(rotSpeed);
    }
}

@Override
public void end(boolean interrupted) {
    armControl.setRotSpeed(0);
}

@Override
public boolean isFinished(){
    return false;
}
}

/* 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControl;
import edu.wpi.first.math.controller.PIDController;

//arm stuff
public class Arm extends CommandBase{

    private ArmControl armControl;
    private double desiredPositionR, rotSpeed;
    

public Arm (ArmControl armControl){
    this.armControl = armControl;
    addRequirements(armControl);
}    

@Override
public void initialize(){

}

public void setRotSpeed(double rotSpeed) {
    this.rotSpeed = rotSpeed;
}

public void setdesiredPositionR(double desiredPositionR) {
    this.desiredPositionR = desiredPositionR;
}

public void GoToPos() {
    if( desiredPositionR != 0){
        armControl.setdesiredPositionR(desiredPositionR);
    } else {
        armControl.setRotSpeed(rotSpeed);
    }
}

@Override
public void execute(){
    GoToPos();
}

@Override
public void end(boolean interrupted) {
    armControl.setRotSpeed(0);
}

@Override
public boolean isFinished(){
    return false;
}
}
 */