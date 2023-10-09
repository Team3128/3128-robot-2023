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

public class NAR_Camera2 extends PhotonCamera {

    public final Camera camera;

    private PhotonPipelineResult result;

    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    private static DoubleSupplier gyro;

    private static BiConsumer<Pose2d, Double> updatePose;

    private static HashMap<Integer, Pose2d> AprilTags;
    public static boolean multipleTargets;

    public static DoubleSupplier thresh;

    public NAR_Camera2(Camera camera) {
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

    public void update() {
        result = this.getLatestResult();
        if (result.hasTargets()) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
            if (!camera.updatePose)
                return;
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            if (multipleTargets) {
                for (int i = 0; i < targets.size(); i++) {
                    if (targetAmbiguity(targets.get(i)) < 0.3 && !getPosApril(targets.get(i)).equals(new Pose2d()))
                        poses.add(getPosApril(targets.get(i)));
                }
            } else if (!getPosApril().equals(new Pose2d()))
                poses.add(getPosApril());
            for (int i = 0; i < poses.size(); i++) {
                if (translationOutOfBounds(poses.get(i).getTranslation()))
                    return;
                updatePose.accept(poses.get(i), result.getTimestampSeconds());
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

    public double targetYaw() {
        return targetYaw(bestTarget);
    }

    private double targetYaw(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getYaw() : 0;
    }

    public double targetPitch() {
        return targetPitch(bestTarget);
    }

    private double targetPitch(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getPitch() : 0;
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
        double hypotenuse = getAprilDistance(target);
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

    public double getAprilDistance() {
        return getAprilDistance(bestTarget);
    }

    // Relative to Camera
    private double getAprilDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget())
            return -1;
        Transform2d transform = getTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    public Pose2d getPosApril() {
        return getPosApril(bestTarget);
    }

    // Relative to Robot
    private Pose2d getPosApril(PhotonTrackedTarget tag) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(tag)))
            return new Pose2d();
        Transform2d transform = getProcessedTarget(tag);
        if (!AprilTags.containsKey(targetId(tag)) || transform.getX() > 5
                || Math.abs(transform.getRotation().getDegrees()) < 150)
            return new Pose2d();
        Pose2d target = AprilTags.get(targetId());
        if (target == null)
            return new Pose2d();
        Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        Rotation2d angle = target.getRotation().plus(transform.getRotation());

        Pose2d pos = new Pose2d(coord, angle);

        return pos.transformBy(camera.offset);
    }

    public String getName() {
        return camera.hostname;
    }

}