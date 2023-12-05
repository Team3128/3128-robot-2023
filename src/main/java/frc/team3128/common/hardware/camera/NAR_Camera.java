package frc.team3128.common.hardware.camera;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Team 3128's streamlined {@link PhotonCamera} class that provides additional functionality and ease of use
 * <p> Geometry: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/index.html
 * 
 * @since 2023 CHARGED UP
 * @author William Yuan, Lucas Han, Audrey Zheng, Mason Lam
 */
public class NAR_Camera extends PhotonCamera {

    public final Camera camera;

    private PhotonPipelineResult result;

    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    private static DoubleSupplier gyro;

    private static BiConsumer<Pose2d, Double> updatePose;

    private static HashMap<Integer, Pose2d> AprilTags;
    public static boolean multipleTargets;

    public static DoubleSupplier thresh;

    private final double FIELD_X_LENGTH = Units.inchesToMeters(648);
    private final double FIELD_Y_LENGTH = Units.inchesToMeters(324);

    /**
     * Creates a NAR_Camera object
     * 
     * @param camera specs of the camera
     */
    public NAR_Camera(Camera camera) {
        super(camera.hostname);
        this.camera = camera;
        setVersionCheckEnabled(false);
    }

    /**
     * Sets the requirements for NAR_Camera
     * 
     * @param angle feeds the angle of the robot
     * @param odometry feeds the robot odometry object for vision estimates to update
     * @param poses sets the AprilTag positions on the field
     * @param haveMultipleTargets represents whether or not to consider multiple targets
     */
    public static void setRequirements(DoubleSupplier angle, BiConsumer<Pose2d, Double> odometry, HashMap<Integer, Pose2d> poses, boolean haveMultipleTargets) {
        gyro = angle;
        updatePose = odometry;
        AprilTags = poses;
        multipleTargets = haveMultipleTargets;
    }

    /**
     * Allows the camera to update robot odometry
     */
    public void enable() {
        camera.updatePose = true;
    }

    /**
     * Stops the camera from updating robot odometry
     */
    public void disable() {
        camera.updatePose = false;
    }

    /**
     * @return the name of the camera
     */
    public String getName() {
        return camera.hostname;
    }

    /**
     * Gets the latest targets and updates the robot position if applicable
     */
    public void update() {
        //returns the most recent camera frame
        result = this.getLatestResult();

        // if camera sees no target, set values to null and return
        if (!result.hasTargets()) {
            targets = null;
            bestTarget = null;
            return;
        }

        targets = result.getTargets();
        bestTarget = result.getBestTarget();

        // if camera is not enabled to update the pose, return
        if (!camera.updatePose) return;

        final ArrayList<Pose2d> possiblePoses = new ArrayList<Pose2d>();

        // add valid targets to possiblePoses
        for (final PhotonTrackedTarget curTarget : targets) {
            final Transform2d transform = getAccTarget(curTarget);

            // if target is tolerable, add to possiblePoses
            if (!getPos(curTarget).equals(new Pose2d()) && targetAmbiguity(curTarget) < 0.5
                && !(getDistance(curTarget) > 5 || Math.abs(transform.getRotation().getDegrees()) < 150)) {

                possiblePoses.add(getPos(curTarget));

                // if camera has multiple targets disabled, break after adding first valid target
                if (!multipleTargets) break;
            }
        }

        // updates robot with all acceptable poses from possiblePoses
        for (final Pose2d curPos : possiblePoses) {
            if (translationOutOfBounds(curPos.getTranslation())) return;
            updatePose.accept(curPos, result.getTimestampSeconds());
        }
    }    

    /**
     * @param translation a calculated translation
     * @return if the Translation2d is within the bounds of the field
     */
    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    /**
     * @return the target ID of the best target represented as a number
     */
    public int targetId() {
        return targetId(bestTarget);
    }

    /**
     * @param target an AprilTag
     * @return the target ID of a target represented as a number
     */
    private int targetId(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getFiducialId() : 0;
    }
    /**
     * The ambiguity of the best target with lower being more accurate
     * @return a value from 0.0 to 1.0 representing the accuracy of the target
     */
    public double targetAmbiguity() {
        return targetAmbiguity(bestTarget);
    }

    /**
     * The ambiguity of a target with lower being more accurate
     * @param target an AprilTag
     * @return a value from 0.0 to 1.0 representing the accuracy of the target
     */
    private double targetAmbiguity(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPoseAmbiguity() : 0;
    }

    /**
     * Coordinate system relative to the camera
     * @return the best target to camera vector as a Transform3d
     */
    public Transform3d getTarget3d() {
        return getTarget3d(bestTarget);
    }

    /**
     * Coordinate system relative to the camera
     * @param target an AprilTag
     * @return a target to camera vector as a Transform3d
     */
    private Transform3d getTarget3d(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    /**
     * Coordinate system relative to the camera
     * @return the best target to camera vector as a Transform2d
     */
    public Transform2d getRelTarget() {
        return getRelTarget(bestTarget);
    }

    /**
     * Coordinate system relative to the camera
     * @param target an AprilTag
     * @return a target to camera vector as a Transform2d
     */
    private Transform2d getRelTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Transform2d();

        final Transform3d transform = getTarget3d(target);

        // unary minus used to reverse the sign of the angle
        return new Transform2d(transform.getTranslation().toTranslation2d(),
                Rotation2d.fromDegrees(MathUtil.inputModulus(transform.getRotation().toRotation2d().unaryMinus().getDegrees(),-180,180)));
    }

    /**
     * Coordinate system relative to the target
     * @return the best target to camera vector as a Transform2d
     */
    public Transform2d getAccTarget() {
        return getAccTarget(bestTarget);
    }

    /**
     * Coordinate system relative to the target
     * @param target an AprilTag
     * @return a target to camera vector as a Transform2d
     */
    private Transform2d getAccTarget(PhotonTrackedTarget target) {
        // if no valid target, return empty Transform2d
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();

        // angle of the AprilTag relative to the field
        final double fieldTargetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        
        // vector relative to camera coordinate system
        Translation2d vector = getRelTarget(target).getTranslation();
        
        // rotated vector to match target coordinate system
        vector = vector.rotateBy(Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getAsDouble() + fieldTargetAngle + camera.offset.getRotation().getDegrees(),-180,180)));

        // angle of the AprilTag relative to the camera
        final Rotation2d relTargetAngle = getRelTarget(target).getRotation();

        return new Transform2d(vector, relTargetAngle);
    }

    /**
     * @return if camera sees any targets
     */
    public boolean hasValidTarget() {
        return targets != null;
    }

    /**
     * @return the distance from the best target to the camera
     */
    public double getDistance() {
        return getDistance(bestTarget);
    }

    /**
     * @param target an April Tag
     * @return the distance from a target to the camera
     */
    private double getDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;

        final Transform2d transform = getRelTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    /**
     * Coordinate system relative to the field
     * @return position of the robot on the field as a Pose2d calculated from the best target
     */
    public Pose2d getPos() {
        return getPos(bestTarget);
    }

    /**
     * Coordinate system relative to the field
     * @param tag an April Tag
     * @return position of the robot on the field as a Pose2d calculated from a target
     */
    private Pose2d getPos(PhotonTrackedTarget tag) {
        final Pose2d target = AprilTags.get(targetId(tag));

        // if no valid target, return empty Pose2d
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(tag)) || target == null) return new Pose2d();

        // vector from target to camera rel to target coordinate system
        final Transform2d transform = getAccTarget(tag);

        // vector from field origin to camera rel to field coordinate system
        final Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));

        // angle of the camera rel to field coordinate system
        final Rotation2d angle = target.getRotation().plus(transform.getRotation());

        // turn field origin to camera vector to field origin to robot vector 
        return new Pose2d(coord, angle).transformBy(camera.offset);
    }

        
    
}