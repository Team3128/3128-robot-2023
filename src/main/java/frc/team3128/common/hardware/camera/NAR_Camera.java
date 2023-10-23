package frc.team3128.common.hardware.camera;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static frc.team3128.Constants.VisionConstants.*;

/**
 * Team 3128's streamlined {@link PhotonCamera} class. Provides additional functionality and ease of use.
 * 
 * @since 2023 CHARGED UP
 * @author Mason Lam, William Yuan, Audrey Zheng, Lucas Han
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

    /**
     * Creates a new object to represent a camera. Disables version checking.
     * 
     * @param camera {@link Camera} object to be represented
     * @see for geometry help: https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/math/geometry/
     */
    public NAR_Camera(Camera camera) {
        super(camera.hostname);
        this.camera = camera;
        setVersionCheckEnabled(false);
    }
    /**
     * Sets the requirements for the camera.
     * 
     * @param angle DoubleSupplier that represents Swerve.getYaw(). Returns the rotation of the robot.
     * @param odometry BiConsumer that represents Swerve.addVisionMeasurement(Pose2d, double). Updates the pose of the robot.
     * @param poses HashMap of poses of the AprilTags.
     * @param haveMultipleTargets boolean representing whether or not to consider multiple targest.
     */
    public static void setRequirements(DoubleSupplier angle, BiConsumer<Pose2d, Double> odometry, HashMap<Integer, Pose2d> poses, boolean haveMultipleTargets) {
        gyro = angle;
        updatePose = odometry;
        AprilTags = poses;
        multipleTargets = haveMultipleTargets;
    }
    /**
     * Enables the camera to update the pose of the robot.
     */
    public void enable() {
        camera.updatePose = true;
    }
    /**
     * Disables the camera from updating the pose of the robot.
     */
    public void disable() {
        camera.updatePose = false;
    }
    /**
     * Returns the name of the camera.
     */
    public String getName() {
        return camera.hostname;
    }
    /**
     * Updates the pose of the robot based on what the camera sees.
     * 
     * @see #update() comments in code for logic.
     */
    public void update() {
        result = this.getLatestResult();

        // if the camera sees no target, set values to null and return
        if (!result.hasTargets()) {
            targets = null;
            bestTarget = null;
            return;
        }

        targets = result.getTargets();
        bestTarget = result.getBestTarget();

        // do not update pose if the camera is not supposed to
        if (!camera.updatePose) return;

        // if the camera sees multiple targets, add all valid targets to poses
        // if the camera sees one target, only add first valid target to poses
        final ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (final PhotonTrackedTarget curTarget : targets) {
            final Transform2d transform = getAccTarget(curTarget);
            // if the target is not ambiguous, the target is not empty, and the target is in an accurate position, add the poses
            if (targetAmbiguity(curTarget) < 0.5 && !getPos(curTarget).equals(new Pose2d())
                && !(getDistance() > 5 || Math.abs(transform.getRotation().getDegrees()) < 150)) {
                poses.add(getPos(curTarget));
                // if only one target, break
                if (!multipleTargets) break;
            }
        }

        // updates Swerve.addVisionMeasurement(Pose2d, double) with all valid poses
        for (final Pose2d curPos : poses) {
            if (translationOutOfBounds(curPos.getTranslation())) return;
            updatePose.accept(curPos, result.getTimestampSeconds());
        }
    }    

    /**
     * Returns if a calculated Pose2d is within the bounds of the field.
     * 
     * @param translation Translation2d of the Pose2d.
     * @return boolean representing whether or not the Pose2d is within the bounds of the field.
     */
    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    /**
     * Returns the target ID of the best target.
     * 
     * @return the target ID of the best target.
     */
    public int targetId() {
        return targetId(bestTarget);
    }

    /**
     * Returns the target ID of a target.
     * @param target PhotonTrackedTarget target to get the ID of.
     * @return the target ID of a target.
     */
    private int targetId(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getFiducialId() : 0;
    }
    /**
     * Returns the error of the best target.
     * @return the error of the best target.
     */
    public double targetAmbiguity() {
        return targetAmbiguity(bestTarget);
    }

    /**
     * Returns the error of a target.
     * @param target PhotonTrackedTarget target to get the error of.
     * @return the error of a target.
     */
    private double targetAmbiguity(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPoseAmbiguity() : 0;
    }

    /**
     * Returns the position of the best target relative to the camera as a Transform3d.
     * @return the position of the best target relative to the camera as a Transform3d.
     */
    public Transform3d getTarget3d() {
        return getTarget3d(bestTarget);
    }

    /**
     * Returns the position of a target relative to the camera as a Transform3d.
     * @param target PhotonTrackedTarget target to get the position of.
     * @return the position of a target relative to the camera as a Transform3d.
     */
    private Transform3d getTarget3d(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    /**
     * Returns the position of the best target relative to the camera as a Transform2d.
     * @return the position of the best target relative to the camera as a Transform2d.
     */
    public Transform2d getTarget() {
        return getTarget(bestTarget);
    }

    /**
     * Returns the position of a target relative to the camera as a Transform2d.
     * @param target PhotonTrackedTarget target to get the position of.
     * @return the position of a target relative to the camera as a Transform2d.
     */
    private Transform2d getTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Transform2d();

        final Transform3d transform = getTarget3d(target);
        return new Transform2d(transform.getTranslation().toTranslation2d(),
                transform.getRotation().toRotation2d().unaryMinus());
    }

    /**
     * Returns a more accurate camera pos relative to target as a Transform2d.
     * @return a more accurate camera pos relative to target as a Transform2d.
     * @see #getAccTarget(PhotonTrackedTarget) comments in code for logic.
     */
    public Transform2d getAccTarget() {
        return getAccTarget(bestTarget);
    }

    /**
     * Returns a more accurate camera pos relative to target as a Transform2d.
     * @param target PhotonTrackedTarget target to get the position of.
     * @return a more accurate camera pos relative to target as a Transform2d.
     * @see #getAccTarget(PhotonTrackedTarget) comments in code for logic.
     */
    private Transform2d getAccTarget(PhotonTrackedTarget target) {
        // if no valid target, return empty Transform2d
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();

        // distance from robot to target
        final double hypotenuse = getDistance(target);
        // angle of the robot relative to target
        final Rotation2d angleTargetToCamera = getTarget().getRotation();
        // angle of the april tag relative to the field (0: ->, 180: <-)
        final double angleFieldToTarget = AprilTags.get(targetId(target)).getRotation().getDegrees();
        final double deltaY = hypotenuse * Math.sin(
                Units.degreesToRadians(gyro.getAsDouble() + angleFieldToTarget + camera.offset.getRotation().getDegrees()));
        final Transform2d vector = getTarget(target);
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() - deltaY), angleTargetToCamera);
    }
    /**
     * Returns if the camera sees any targets.
     * 
     * @return boolean returning if List<PhotonTrackedTarget> targets is not empty.
     */
    public boolean hasValidTarget() {
        return targets != null;
    }

    /**
     * Returns the distance from the best target to the camera.
     * 
     * @return the distance from the best target to the camera.
     */
    public double getDistance() {
        return getDistance(bestTarget);
    }

    /**
     * Returns the distance from a target to the camera.
     * @param target PhotonTrackedTarget target to get the distance of.
     * @return the distance from a target to the camera.
     */
    private double getDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;

        final Transform2d transform = getTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    /**
     * Returns the position of the robot, as a Pose2d, relative to the field using the position of the best target.
     * @return the position of the robot, as a Pose2d relative to the field using the position of the best target.
     * @see #getPos(PhotonTrackedTarget) comments in code for logic.
     */
    public Pose2d getPos() {
        return getPos(bestTarget);
    }

    /**
     * Returns the position of the robot, as a Pose2d, relative to the field using the position of a target.
     * @param tag PhotonTrackedTarget target to use to get the position of the robot.
     * @return the position of the robot, as a Pose2d relative to the field using the position of a target.
     * @see #getPos(PhotonTrackedTarget) comments in code for logic.
     */
    private Pose2d getPos(PhotonTrackedTarget tag) {
        final Pose2d target = AprilTags.get(targetId());
        // if no valid target, return empty Pose2d
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(tag)) || target == null) return new Pose2d();

        // transforms the camera pos relative to target to camera pos relative to field
        final Transform2d transform = getAccTarget(tag);
        final Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        final Rotation2d angle = target.getRotation().plus(transform.getRotation());

        // camera pos to robot pos
        return new Pose2d(coord, angle).transformBy(camera.offset);
    }
}