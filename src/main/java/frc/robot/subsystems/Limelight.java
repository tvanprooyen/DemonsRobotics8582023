package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LimelightData;
import frc.robot.Util.LimelightData.BasicTargetInput;
import frc.robot.Util.LimelightData.CameraControlsOutput;

public class Limelight extends SubsystemBase {
    
    private PIDController LimeLightPID;

    private LimelightData llData = new LimelightData();

    private int pipeline;

    public Limelight() {

        LimeLightPID = new PIDController(0.01, 0, 0);

        this.pipeline = 3;
    }

    /**
     * Sets the pipeline
     * @param pipeline 0 = Retro Reflective, 1 = ApirlTags, 3 = Camera
      */
    public void setPipeLine(int pipeline) {
        this.pipeline = pipeline;
        llData.setCameraControls(CameraControlsOutput.pipeline, pipeline);
    }

    public double PIDControllerCalculation() {
        if(this.pipeline != 3) {
            return LimeLightPID.calculate(llData.getBasic(BasicTargetInput.tx), 0);
        }
        return 0;
    }

    private void runLimeLight() {
        PIDControllerCalculation();
    }


    @Override
    public void periodic() {
        runLimeLight();

        dashboard();
    }

    private void dashboard() {
        SmartDashboard.putBoolean("In Camera Mode", this.pipeline == 1);
        SmartDashboard.putNumber("LimeLight PID", PIDControllerCalculation());
    }
}