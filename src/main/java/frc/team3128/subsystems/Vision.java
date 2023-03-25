package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.HashMap;

import static frc.team3128.Constants.VisionConstants.*;

import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Vision extends SubsystemBase{
    public static int SELECTED_GRID = 0;
    public static boolean AUTO_ENABLED = false;
    public static boolean MANUAL = false;
    public static ArmPosition position = ArmPosition.NEUTRAL;

    private HashMap<String,NAR_Camera> cameras;
  
    private static Vision instance;

    public static synchronized Vision getInstance(){
        if(instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        Swerve swerve = Swerve.getInstance();
        NAR_Camera.setGyro(()-> swerve.getYaw());
        NAR_Camera.setOdometry((pose,time) -> swerve.addVisionMeasurement(pose,time));
        NAR_Camera.setAprilTags(APRIL_TAG_POS);
        NAR_Camera.multipleTargets = false;
        cameras = new HashMap<String,NAR_Camera>();
        cameras.put(FRONT.hostname, new NAR_Camera(FRONT));
        cameras.put(BACK.hostname, new NAR_Camera(BACK));
    }

    public Pose2d targetPos(String name, Pose2d robotPos) {
        return cameras.get(name).getTargetPos(robotPos);
    }

    public void visionReset() {
        Swerve swerve = Swerve.getInstance();
        for (NAR_Camera cam : getCameras()) {
            Pose2d pose = cam.getPos();
            if (!pose.equals(new Pose2d())) {
                swerve.resetOdometry(new Pose2d(pose.getTranslation(),swerve.getGyroRotation2d()));
                return;
            }
        }
    }

    public void enableVision() {
        for (NAR_Camera cam : getCameras()) {
            cam.enable();
        }
    }

    public void disableVision() {
        for (NAR_Camera cam : getCameras()) {
            cam.disable();
        }
    }

    public Pose2d robotPos(String name) {
        return cameras.get(name).getPos();
    }

    public double calculateDistance(String name) {
        return cameras.get(name).getDistance();
    }

    public double getTX(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetYaw();
    }

    public double getTY(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetPitch();
    }

    public double getArea(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetArea();
    }

    public boolean hasValidTarget(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.hasValidTarget();
    }

    public void setLED(String name, boolean state) {
        NAR_Camera camera = cameras.get(name);
        camera.setLED(state);
    }

    public NAR_Camera getCamera(String name) {
        return cameras.get(name);
    }

    public NAR_Camera getCamera(Camera camera) {
        return getCamera(camera.hostname);
    }

    public Camera camSpecs(String name) {
        return cameras.get(name).camera;
    }

    public Collection<NAR_Camera> getCameras(){
        return cameras.values();
    }

    @Override
    public void periodic(){
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
    }
    
    public void initShuffleboard() {
        NAR_Camera cam = cameras.get(FRONT.hostname);
        NAR_Camera cam2 = cameras.get(BACK.hostname);
        NAR_Shuffleboard.addData("VisionComp", "HasTarget", ()->cam.hasValidTarget() || cam2.hasValidTarget(), 0, 0);

        NAR_Shuffleboard.addData("Vision","HasTarget", ()->cam.hasValidTarget(), 0, 0);
        NAR_Shuffleboard.addData("Vision","Distance",()->cam.getDistance(),1,0);
        NAR_Shuffleboard.addData("Vision","RawTarget",()->cam.getTarget().toString(),0,1,4,1);
        NAR_Shuffleboard.addData("Vision", "Processed Target",()->cam.getProcessedTarget().toString(),0,2,4,1);
        NAR_Shuffleboard.addData("Vision","EstimatedPose", ()-> cam.getPos().toString(),0,3,4,1);
        NAR_Shuffleboard.addData("Drivetrain", "HasTarget", ()-> cam.hasValidTarget(), 1, 1);
        NAR_Shuffleboard.addData("Test", "Test", ()->SELECTED_GRID,0,0);
        NAR_Shuffleboard.addData("Test", "TESTING", ()->cam.getTest().toString(),0,1,3,1);

        NAR_Shuffleboard.addData("Vision2","HasTarget", ()->cam2.hasValidTarget(), 0, 0);
        NAR_Shuffleboard.addData("Vision2","Distance",()->cam2.getDistance(),1,0);
        NAR_Shuffleboard.addData("Vision2","RawTarget",()->cam2.getTarget().toString(),0,1,4,1);
        NAR_Shuffleboard.addData("Vision2", "Processed Target",()->cam2.getProcessedTarget().toString(),0,2,4,1);
        NAR_Shuffleboard.addData("Vision2","EstimatedPose", ()-> cam2.getPos().toString(),0,3,4,1);
        NAR_Shuffleboard.addData("Test", "TESTING", ()->cam2.getTest().toString(),0,2,3,1);

        NAR_Shuffleboard.addData("Togglables", "AUTO_ENABLED", ()-> AUTO_ENABLED, 0, 0);
        NAR_Shuffleboard.addData("Togglables", "MANUAL", ()-> MANUAL, 1, 0);
    }

    public void logCameraAll() {
        NAR_Shuffleboard.addData("Vision Urgent", "HasTarget", cameras.get(FRONT.hostname).hasValidTarget(),0,0);
        NAR_Shuffleboard.addData("Vision Urgent", "Distance", calculateDistance(FRONT.hostname),1,0);
        NAR_Shuffleboard.addData("Vision Urgent", "EstimatedPose",cameras.get(FRONT.hostname).getPos().toString(),0,1);
        NAR_Shuffleboard.addData("Vision Urgent", "GEICO",cameras.get(FRONT.hostname).targetYaw(),2,0);
        NAR_Shuffleboard.addData("Vision Urgent", "TARGETPOS",cameras.get(FRONT.hostname).getTargetPos(Swerve.getInstance().getPose()).toString(),0,2);
        NAR_Shuffleboard.addData("Vision Urgent", "RAWTARGET",cameras.get(FRONT.hostname).getTarget().toString(),0,3);
        NAR_Shuffleboard.addData("Vision Urgent", "TARGETGUITY",cameras.get(FRONT.hostname).targetAmbiguity(),3,0);
        NAR_Shuffleboard.addData("Vision Urgent", "PROCESSED TARGET", cameras.get(FRONT.hostname).getProcessedTarget().toString(),0,4);
        NAR_Shuffleboard.addData("Vision Urgent", "SUNSHINE", cameras.get(FRONT.hostname).getRawTarget().toString(),0,5);
    }
}