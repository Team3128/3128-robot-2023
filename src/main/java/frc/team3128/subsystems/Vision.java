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

/**
 * Team 3128's vision processing subsystem
 * 
 * @since 2023 CHARGED UP
 * @author William Yuan, Mason Lam
 */
public class Vision extends SubsystemBase {
    public static int SELECTED_GRID = 0;
    public static boolean AUTO_ENABLED = false;
    public static boolean MANUAL = false;
    public static ArmPosition position = ArmPosition.NEUTRAL;

    private HashMap<String, NAR_Camera> cameras;

    private static Vision instance;

    /**
     * @return the singleton instance of the subsystem
     */
    public static synchronized Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    /**
     * Creates a new object to control vision processing
     * <p> Set requirements for NAR_Camera and adds cameras
     */
    public Vision() {
        Swerve swerve = Swerve.getInstance();
        NAR_Camera.setRequirements(() -> swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), APRIL_TAG_POS, false);

        cameras = new HashMap<String, NAR_Camera>();
        cameras.put(FRONT.hostname, new NAR_Camera(FRONT));
        cameras.put(BACK.hostname, new NAR_Camera(BACK));
    }

    /**
     * Resets and corrects the robot's calculated position on the field 
     */
    public void visionReset() {
        Swerve swerve = Swerve.getInstance();
        for (NAR_Camera cam : getCameras()) {

            Pose2d pose = cam.getPos();
            if (!pose.equals(new Pose2d())) {
                swerve.resetOdometry(new Pose2d(pose.getTranslation(), swerve.getGyroRotation2d()));
                return;
            }
        }
    }

    /**
     * Enables vision processing
     */
    public void enableVision() {
        for (NAR_Camera cam : getCameras()) {
            cam.enable();
        }
    }

    /**
     * Disables vision processing
     */
    public void disableVision() {
        for (NAR_Camera cam : getCameras()) {
            cam.disable();
        }
    }

    /**
     * @param name the name of the camera
     * @return the robot's position on the field calculated using the camera
     */
    public Pose2d robotPos(String name) {
        return cameras.get(name).getPos();
    }

    /**
     * @param name the name of the camera
     * @return the distance from the target to the camera
     */
    public double calculateDistance(String name) {
        return cameras.get(name).getDistance();
    }
    /**
     * @param name the name of the camera
     * @return if the camera sees a target
     */
    public boolean hasValidTarget(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.hasValidTarget();
    }
    /**
     * @param name the name of the camera
     * @return the NAR_Camera instance of that camera
     */
    public NAR_Camera getCamera(String name) {
        return cameras.get(name);
    }

    /**
     * @param name the name of the camera
     * @return the specs of the camera
     */
    public Camera camSpecs(String name) {
        return cameras.get(name).camera;
    }

    /**
     * @return a collection of all the cameras
     */
    public Collection<NAR_Camera> getCameras() {
        return cameras.values();
    }

    /**
     * Updates the robot's calculated position on the field
     */
    @Override
    public void periodic() {
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
    }

    /**
     * Logs all desired data from the cameras to NAR_Shuffleboard
     */
    public void initShuffleboard() {
        NAR_Camera cam = cameras.get(FRONT.hostname);
        NAR_Camera cam2 = cameras.get(BACK.hostname);

        NAR_Shuffleboard.addData("Vision", "HasTarget", () -> cam.hasValidTarget(), 0, 0);
        NAR_Shuffleboard.addData("Vision", "Distance", () -> cam.getDistance(), 1, 0);
        
        NAR_Shuffleboard.addData("Vision", "RelTarget", () -> cam.getRelTarget().toString(), 0, 1, 4, 1);
        NAR_Shuffleboard.addData("Vision", "Acc Target", () -> cam.getAccTarget().toString(), 0, 2, 4, 1);
        NAR_Shuffleboard.addData("Vision", "EstimatedPose", () -> cam.getPos().toString(), 0, 4, 4, 1);
        
        NAR_Shuffleboard.addData("Vision2", "HasTarget", () -> cam2.hasValidTarget(), 0, 0);
        NAR_Shuffleboard.addData("Vision2", "Distance", () -> cam2.getDistance(), 1, 0);
        NAR_Shuffleboard.addData("Vision2", "RelTarget", () -> cam2.getRelTarget().toString(), 0, 1, 4, 1);
         NAR_Shuffleboard.addData("Vision2", "Acc Target", () -> cam2.getAccTarget().toString(), 0, 2, 4, 1);
        NAR_Shuffleboard.addData("Vision2", "EstimatedPose", () -> cam2.getPos().toString(), 0, 4, 4, 1);

        NAR_Shuffleboard.addData("Togglables", "AUTO_ENABLED", () -> AUTO_ENABLED, 0, 0);
        NAR_Shuffleboard.addData("Togglables", "MANUAL", () -> MANUAL, 1, 0);

        NAR_Shuffleboard.addData("Drivetrain", "HasTarget", () -> cam.hasValidTarget(), 1, 1);
        NAR_Shuffleboard.addData("Test", "Test", () -> SELECTED_GRID, 0, 0);
        NAR_Shuffleboard.addData("VisionComp", "HasTarget", () -> cam.hasValidTarget() || cam2.hasValidTarget(), 0, 0);
    }
    
    /**
     * Logs all necessary data from the cameras to NAR_Shuffleboard
     */
    public void logCameraAll() {
        NAR_Shuffleboard.addData("Vision Urgent", "HasTarget", cameras.get(FRONT.hostname).hasValidTarget(), 0, 0);
        NAR_Shuffleboard.addData("Vision Urgent", "Distance", calculateDistance(FRONT.hostname), 1, 0);
        NAR_Shuffleboard.addData("Vision Urgent", "EstimatedPose", cameras.get(FRONT.hostname).getPos().toString(), 0,
                1);
        NAR_Shuffleboard.addData("Vision Urgent", "RAWTARGET", cameras.get(FRONT.hostname).getRelTarget().toString(), 0,
                3);
        NAR_Shuffleboard.addData("Vision Urgent", "TARGETGUITY", cameras.get(FRONT.hostname).targetAmbiguity(), 3, 0);
        NAR_Shuffleboard.addData("Vision Urgent", "PROCESSED TARGET",
                cameras.get(FRONT.hostname).getAccTarget().toString(), 0, 4);
        NAR_Shuffleboard.addData("Vision Urgent", "SUNSHINE", cameras.get(FRONT.hostname).getTarget3d().toString(), 0,
                5);
    }
}