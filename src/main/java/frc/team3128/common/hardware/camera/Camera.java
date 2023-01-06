package frc.team3128.common.hardware.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum Camera {
    SHOOTER("Frog",true,0,0, 0, new Transform2d(new Translation2d(Units.inchesToMeters(-12),0), Rotation2d.fromDegrees(180))), //Bad Numbers
    PI_CAMERA("teja",false,10000, 40, 0,new Transform2d(new Translation2d(10,0), Rotation2d.fromDegrees(0))); //Bad Numbers

    public String hostname;

    public boolean LED;

    public double height; //inches

    public double angle;  //degrees

    public double targetHeight; //inches

    public Transform2d offset; //inches

    private Camera(String hostname, boolean LED, double cameraHeight, double cameraAngle, double targetHeight, Transform2d cameraOffset) {
        this.hostname = hostname;
        this.LED = LED;
        this.height = cameraHeight;
        this.angle = cameraAngle;
        this.targetHeight = targetHeight;
        this.offset = cameraOffset;
    }

}
