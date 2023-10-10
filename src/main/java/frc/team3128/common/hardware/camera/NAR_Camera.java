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

    public NAR_Camera(Camera camera) {
        super(camera.hostname);
        this.camera = camera;
        setVersionCheckEnabled(false);
    }

    public static void setRequirements(DoubleSupplier angle, BiConsumer<Pose2d, Double> odometry,
            HashMap<Integer, Pose2d> poses, boolean haveMultipleTargets) {
        gyro = angle;
        updatePose = odometry;
        AprilTags = poses;
        multipleTargets = haveMultipleTargets;
    }

    public void enable() {
        camera.updatePose = true;
    }

    public void disable() {
        camera.updatePose = false;
    }

    //TODO: Clean up this method
    public void update() {
        result = this.getLatestResult();

        if (result.hasTargets()) {

            targets = result.getTargets();
            bestTarget = result.getBestTarget();

            if (!camera.updatePose) return;

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            if (multipleTargets) {
                for (PhotonTrackedTarget curTarget : targets) {
                    if (targetAmbiguity(curTarget) < 0.3 && !getApril(curTarget).equals(new Pose2d()))
                        poses.add(getApril(curTarget));
                }
            } else if (!getPos().equals(new Pose2d()))
                poses.add(getPos());

            for (Pose2d curPos : poses) {
                if (translationOutOfBounds(curPos.getTranslation()))
                    return;
                updatePose.accept(curPos, result.getTimestampSeconds());
            }
            return;
        }

        targets = null;
        bestTarget = null;
    }

    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    public int targetId() {
        return targetId(bestTarget);
    }

    private int targetId(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getFiducialId() : 0;
    }

    public double targetAmbiguity() {
        return targetAmbiguity(bestTarget);
    }

    private double targetAmbiguity(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPoseAmbiguity() : 0;
    }

    public Transform3d getRawTarget() {
        return getRawTarget(bestTarget);
    }

    private Transform3d getRawTarget(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    public Transform2d getTarget() {
        return getTarget(bestTarget);
    }

    // rawTarget() + transform
    private Transform2d getTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget())
            return new Transform2d();
        Transform3d transform = getRawTarget(target);
        return new Transform2d(transform.getTranslation().toTranslation2d(),
                transform.getRotation().toRotation2d().unaryMinus());
    }

    public Transform2d getProcessedTarget() {
        return getProcessedTarget(bestTarget);
    }

    // process getTarget()
    private Transform2d getProcessedTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target)))
            return new Transform2d();
        double hypotenuse = getDistance(target);
        Rotation2d angle = getTarget().getRotation();
        double targetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        double deltaY = hypotenuse * Math.sin(
                Units.degreesToRadians(gyro.getAsDouble() + targetAngle + camera.offset.getRotation().getDegrees()));
        Transform2d vector = getTarget(target);
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() - deltaY), angle);
    }

    public boolean hasValidTarget() {
        return targets != null;
    }

    public double getDistance() {
        return getDistance(bestTarget);
    }

    // Relative to Camera
    private double getDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget())
            return -1;
        Transform2d transform = getTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    public Pose2d getPos() {
        return getApril(bestTarget);
    }

    // TODO: fix
    // Relative to Robot
    private Pose2d getApril(PhotonTrackedTarget tag) {
        Pose2d target = AprilTags.get(targetId());
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(tag)) || target == null)
            return new Pose2d();

        Transform2d transform = getProcessedTarget(tag);
        Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        Rotation2d angle = target.getRotation().plus(transform.getRotation());

        Pose2d pos = new Pose2d(coord, angle).transformBy(camera.offset);

        

        if (transform.getX() > 5 || Math.abs(transform.getRotation().getDegrees()) < 150) {
            return new Pose2d();
        }
        return pos;
    }

    public String getName() {
        return camera.hostname;
    }

}