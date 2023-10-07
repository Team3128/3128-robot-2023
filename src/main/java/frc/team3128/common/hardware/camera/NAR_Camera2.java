package frc.team3128.common.hardware.camera;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.team3128.Constants.VisionConstants.*;

public class NAR_Camera2 extends PhotonCamera {

    public final Camera camera;

    private PhotonPipelineResult result;

    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    private static DoubleSupplier gyro;
    
    private static BiConsumer<Pose2d,Double> updatePose;

    private static HashMap<Integer, Pose2d> AprilTags;
    private static Pose2d visionTarget;
    public static boolean multipleTargets = false;

    public static DoubleSupplier thresh;

    public NAR_Camera2(Camera camera) {
        super(camera.hostname);
        this.camera = camera;
        setLED(false);
        setVersionCheckEnabled(false);
    }

    public static void setRequirements(DoubleSupplier angle, BiConsumer<Pose2d,Double> odometry, HashMap<Integer,Pose2d> poses, boolean haveMultipleTargets){
        gyro = angle;
        updatePose = odometry;
        AprilTags = poses;
        multipleTargets = haveMultipleTargets;
    }

    public static void setGyro(DoubleSupplier angle) {
        gyro = angle;
    }

    public static void setOdometry(BiConsumer<Pose2d,Double> odometry) {
        updatePose = odometry;
    }

    public static void setAprilTags(HashMap<Integer, Pose2d> poses) {
        AprilTags = poses;
    }

    public static void setVisionTarget(Pose2d pose) {
        visionTarget = pose;
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
            if (!camera.updatePose) return;
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            if (multipleTargets) {
                for (int i = 0; i < targets.size(); i ++) {
                    if (targetAmbiguity(targets.get(i)) < 0.3 && !getPos(targets.get(i)).equals(new Pose2d()))
                        poses.add(getPos(targets.get(i)));
                }
            }
            else if (!getPos().equals(new Pose2d()))poses.add(getPos());
            for (int i = 0; i < poses.size(); i++) {
                if (translationOutOfBounds(poses.get(i).getTranslation()))
                    return;
                updatePose.accept(poses.get(i),result.getTimestampSeconds());
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

    public double targetArea() {
        return targetArea(bestTarget);
    }

    private double targetArea(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getArea() : 0;
    }

    public double targetSkew() {
        return targetSkew(bestTarget);
    }

    private double targetSkew(PhotonTrackedTarget target) {
        return hasValidTarget() ? target.getSkew() : 0;
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
    
    private Transform2d getTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Transform2d();
        Transform3d transform = getRawTarget(target);
        return new Transform2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d().unaryMinus());
    }

    public Transform2d getProcessedTarget() {
        return getProcessedTarget(bestTarget);
    }

    private Transform2d getProcessedTarget(PhotonTrackedTarget target) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();
        double hypotenuse = getAprilDistance(target);
        Rotation2d angle = getTarget().getRotation();
        double targetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        double deltaY = hypotenuse * Math.sin(Units.degreesToRadians(gyro.getAsDouble() + targetAngle + camera.offset.getRotation().getDegrees()));
        Transform2d vector = getTarget(target);
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() - deltaY), angle);
    }

    public Transform2d getTest() {
        return getTest(bestTarget);
    }

    private Transform2d getTest(PhotonTrackedTarget target) {
        if (!hasValidTarget() || !AprilTags.containsKey(targetId(target))) return new Transform2d();
        Rotation2d angle = getTarget().getRotation();
        double targetAngle = AprilTags.get(targetId(target)).getRotation().getDegrees();
        Transform2d vector = getTarget(target);
        double deltaY = vector.getX() * Math.tan(Units.degreesToRadians(gyro.getAsDouble() + targetAngle));
        return new Transform2d(new Translation2d(vector.getX(), vector.getY() + deltaY), angle);
    }
    

    public boolean hasValidTarget() {
        return targets != null;
    }

    public double getDistance() {
        return getAprilDistance(bestTarget);
    }

    // Relative to Camera
    private double getAprilDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;
        Transform2d transform = getTarget(target);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    // Relative to Camera
    private double getVisionDistance(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return -1;
        double ty = Units.degreesToRadians(targetPitch(target) + camera.angle);
        double tx = Units.degreesToRadians(targetYaw(target));
        return Math.abs(camera.targetHeight - camera.height) / (Math.tan(ty) * Math.cos(tx));
    }

    public void setLED(boolean state) {
        setLED(state ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    public Pose2d getTargetPos(Pose2d robotPos) {
        return getTargetPosApril(robotPos, bestTarget);
    }

    private Pose2d getTargetPosApril(Pose2d robotPos, PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Transform2d transform = getTarget(target).inverse();
        return cameraPos.plus(transform);
    }

    private Pose2d getTargetPosVision(Pose2d robotPos, PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Double distance = getVisionDistance(target);
        Double angle = cameraPos.getRotation().getRadians() - Units.degreesToRadians(targetYaw(target));
        return new Pose2d(cameraPos.getX() + distance * Math.cos(angle), cameraPos.getY() + distance * Math.sin(angle),
                new Rotation2d(angle));
    }

    public Pose2d getPos() {
        return getPos(bestTarget);
    }

    private Pose2d getPos(PhotonTrackedTarget target) {
        return getPosApril(target);
    }

    // Relative to Robot
    private Pose2d getPosApril(PhotonTrackedTarget tag) {
        if(!hasValidTarget() || !AprilTags.containsKey(targetId(tag))) return new Pose2d();
        Transform2d transform = getProcessedTarget(tag);
        if (!AprilTags.containsKey(targetId(tag)) || transform.getX() > 5 || Math.abs(transform.getRotation().getDegrees()) < 150) return new Pose2d();
        Pose2d target = AprilTags.get(targetId());
        if (target == null) return new Pose2d();
        Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        Rotation2d angle = target.getRotation().plus(transform.getRotation());

        Pose2d pos = new Pose2d(coord,angle);

        return pos.transformBy(camera.offset);
    }

    // Relative to Robot
    private Pose2d getPosVision(PhotonTrackedTarget target) {
        if (!hasValidTarget()) return new Pose2d();
        double distance = getVisionDistance(target);
        double yaw = Units.degreesToRadians(targetYaw(target));
        Translation2d translation = new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));

        Transform2d transform = new Transform2d(translation,
                new Rotation2d(Units.degreesToRadians(gyro.getAsDouble())));
        Pose2d pos = visionTarget.transformBy(transform.inverse());
        return pos.transformBy(camera.offset);
    }

    // Relative to Robot
    @Deprecated
    private Pose2d visionEstimatedPose(Pose2d pose) {
        if (!hasValidTarget())
            return new Pose2d();
        double distToHubCenter = getDistance();
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyro.getAsDouble() - targetYaw());
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()),
                -distToHubCenter * Math.sin(thetaHub.getRadians()))
                .plus(pose.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(gyro.getAsDouble()));
        pos.transformBy(camera.offset);

        return pos;
    }

    // Relative to Robot
    @Deprecated
    private Pose2d visionPos(Pose2d pose) {
        if (!hasValidTarget())
            return new Pose2d();
        double distToHubCenter = getDistance();
        double angle = Units.degreesToRadians(gyro.getAsDouble() - targetYaw() - 180);
        Translation2d fieldPos = new Translation2d(distToHubCenter * Math.cos(angle), distToHubCenter * Math.sin(angle))
                .plus(pose.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(gyro.getAsDouble()));
        pos.transformBy(camera.offset);

        return pos;
    } 

    public String get_name() {
        return camera.hostname;
    }

}