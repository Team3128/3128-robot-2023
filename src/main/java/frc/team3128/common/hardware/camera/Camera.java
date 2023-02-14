package frc.team3128.common.hardware.camera;

import edu.wpi.first.math.geometry.Transform2d;

public class Camera {

    public String hostname;

    public boolean updatePose;
  
    public double height; //inches

    public double angle;  //degrees

    public double targetHeight; //inches

    public Transform2d offset; //inches


    public Camera(String hostname, boolean updatePose, double cameraHeight, double cameraAngle, double targetHeight, Transform2d cameraOffset) {
        this.hostname = hostname;
        this.updatePose = updatePose;
        this.height = cameraHeight;
        this.angle = cameraAngle;
        this.targetHeight = targetHeight;
        this.offset = cameraOffset;
    }

}