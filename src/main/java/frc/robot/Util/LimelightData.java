package frc.robot.Util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightData {

    /** 
     * @param tv	  Whether the limelight has any valid targets (0 or 1)
     * @param tx	  Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     * @param ty	  Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     * @param ta	  Target Area (0% of image to 100% of image)
     * @param tl	  The pipeline’s latency contribution (ms). Add to “cl” to get total latency.
     * @param cl	  Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
     * @param tshort  Sidelength of shortest side of the fitted bounding box (pixels)
     * @param tlong	  Sidelength of longest side of the fitted bounding box (pixels)
     * @param thor	  Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * @param tvert	  Vertical sidelength of the rough bounding box (0 - 320 pixels)
     * @param getpipe True active pipeline index of the camera (0 .. 9)
     * @param json	  Full JSON dump of targeting results
     * @param tclass  Class ID of primary neural detector result or neural classifier result
     * 
     * <b>Limelight Docs - Basic Targeting Data</b>
     * https://docs.limelightvision.io/en/latest/networktables_api.html#basic-targeting-data
     */
    public enum BasicTargetInput {
        tv,
        tx,
        ty,
        ta,
        tl,
        cl,
        tshort,
        tlong,
        thor,
        tvert,
        getpipe,
        json,
        tclass
    }

    /** 
     * @param botpose	            Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     * @param botpose_wpiblu         Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     * @param botpose_wpired	        Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     * @param camerapose_targetspace	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
     * @param targetpose_cameraspace	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
     * @param targetpose_robotspace	3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
     * @param botpose_targetspace	3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
     * @param camerapose_robotspace	3D transform of the camera in the coordinate system of the robot (array (6))
     * @param tid	                ID of the primary in-view AprilTag
     * 
     * <b>Limelight Docs - ApirlTag and 3D Data</b>
     * https://docs.limelightvision.io/en/latest/networktables_api.html#apriltag-and-3d-data
     */
    public enum ApirlTagInput {
        botpose,
        botpose_wpiblue,
        botpose_wpired,
        camerapose_targetspace,
        targetpose_cameraspace,
        targetpose_robotspace,
        botpose_targetspace,
        camerapose_robotspace,
        tid
    }

    /** 
     * @param ledMode	Sets limelight’s LED state
     * @param camMode	Sets limelight’s operation mode
     * @param pipeline	Sets limelight’s current pipeline
     * @param snapshot	Allows users to take snapshots during a match
     * 
     * <b>Limelight Docs - Camera Controls</b>
     * https://docs.limelightvision.io/en/latest/networktables_api.html#camera-controls
     */
    public enum CameraControlsOutput {
        ledMode,
        camMode,
        pipeline,
        stream,
        snapshot
    }
    
    /** 
     * @param bti BasicTartgetInput is a all the settings in a group
     * @return double from limelight
     */
    public double getBasic(BasicTargetInput bti) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(bti.name()).getDouble(0);
    }

    /** 
     * @param ati ApirlTagInput is a all the settings in a group
     * @return double array from limelight
     */
    public double[] getApirlTag(ApirlTagInput ati) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(ati.name()).getDoubleArray(new double[6]);
    }

    public void setCameraControls(CameraControlsOutput cco, double setting) {

        NetworkTableInstance.getDefault().getTable("limelight").getEntry(cco.name()).setNumber(setting);
    }

    public void setCrop(double x0, double x1, double y0, double y1) {
        double[] cropValues = new double[4];
        cropValues[0] = x0;
        cropValues[1] = x1;
        cropValues[2] = y0;
        cropValues[3] = y1;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropValues);
    }
}