package frc.team3128.common.hardware.camera;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.Constants.FieldConstants.*;

public class NAR_Camera extends PhotonCamera {

    public enum Pipeline {
        APRILTAG(1),
        RED(2),
        BLUE(3),
        Green(4);

        public int index;

        private Pipeline(int index) {
            this.index = index;
        }
    }

    public final Camera camera;

    private PhotonPipelineResult result;

    private PhotonTrackedTarget target;

    private Swerve swerve;

    private Pipeline sPipeline;

    public NAR_Camera(Camera camera) {
        this(camera, Pipeline.APRILTAG);
    }

    public NAR_Camera(Camera camera, Pipeline sPipeline) {
        super(camera.hostname);
        this.camera = camera;
        swerve = Swerve.getInstance();
        setLED(false);
        setVersionCheckEnabled(false);
        this.sPipeline = sPipeline;
    }

    public void update() {
        result = this.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            Pose2d estimatedPose = getPos();
            // if (sPipeline.equals(Pipeline.APRILTAG) && targetAmbiguity() < 0.2)
            //     return;
            if (translationOutOfBounds(estimatedPose.getTranslation()))
                return;
            double estimatedAngle = MathUtil.inputModulus(estimatedPose.getRotation().getDegrees(), -180, 180);
            double gyroAngle = MathUtil.inputModulus(swerve.getHeading(), -180, 180);
            gyroAngle = estimatedAngle;
            System.out.println("1");
            if (Math.abs(estimatedAngle - gyroAngle) > ANGLE_THRESHOLD)
                return;
            System.out.println("Pain");
            swerve.odometry.addVisionMeasurement(estimatedPose,result.getTimestampSeconds() - result.getLatencyMillis() / 1000);
            return;
        }
        target = null;
    }

    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH
                || translation.getY() < 0;
    }

    public double targetYaw() {
        return hasValidTarget() ? target.getYaw() : 0;
    }

    public double targetPitch() {
        return hasValidTarget() ? target.getPitch() : 0;
    }

    public double targetArea() {
        return hasValidTarget() ? target.getArea() : 0;
    }

    public double targetSkew() {
        return hasValidTarget() ? target.getSkew() : 0;
    }

    public int targetId() {
        return hasValidTarget() ? target.getFiducialId() : -1;
    }

    public double targetAmbiguity() {
        return hasValidTarget() ? target.getPoseAmbiguity() : -1;
    }

    public Transform3d getRawTarget() {
        return hasValidTarget() ? target.getBestCameraToTarget() : new Transform3d();
    }

    public Transform2d getTarget() {
        Transform3d target = getRawTarget();
        return new Transform2d(target.getTranslation().toTranslation2d(), target.getRotation().toRotation2d());
    }

    // public List<TargetCorner> targetCorners() {
    //     return hasValidTarget() ? target.getCorners() : new ArrayList<TargetCorner>();
    // }

    public boolean hasValidTarget() {
        return target != null;
    }

    public double getDistance() {
        return sPipeline.equals(Pipeline.APRILTAG) ? getAprilDistance() : getVisionDistance();
    }

    // Relative to Camera
    private double getAprilDistance() {
        if (!hasValidTarget())
            return -1;
        Transform2d transform = getTarget();
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    // Relative to Camera
    private double getVisionDistance() {
        if (!hasValidTarget())
            return -1;
        double ty = Units.degreesToRadians(targetPitch() + camera.angle);
        double tx = Units.degreesToRadians(targetYaw());
        return Math.abs(camera.targetHeight - camera.height) / (Math.tan(ty) * Math.cos(tx));
    }

    public void setLED(boolean state) {
        if (!camera.LED)
            return;
        setLED(state ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    public Pose2d getTargetPos(Pose2d robotPos) {
        return sPipeline.equals(Pipeline.APRILTAG) ? getTargetPosApril(robotPos) : getTargetPosVision(robotPos);
    }

    private Pose2d getTargetPosApril(Pose2d robotPos) {
        if (!hasValidTarget())
            return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Transform2d target = getTarget().inverse();
        return cameraPos.plus(target);
    }

    private Pose2d getTargetPosVision(Pose2d robotPos) {
        if (!hasValidTarget())
            return new Pose2d();
        Pose2d cameraPos = robotPos.transformBy(camera.offset.inverse());
        Double distance = getDistance();
        Double angle = cameraPos.getRotation().getRadians() - Units.degreesToRadians(targetYaw());
        return new Pose2d(cameraPos.getX() + distance * Math.cos(angle), cameraPos.getY() + distance * Math.sin(angle),
                new Rotation2d(angle));
    }

    public Pose2d getPos() {
        return sPipeline.equals(Pipeline.APRILTAG) ? getPosApril() : getPosVision();
    }

    // Relative to Robot
    private Pose2d getPosApril() {
        if(!hasValidTarget() || targetId() >= APRIL_TAG_POS.length) return new Pose2d();
        Transform2d transform = getTarget();
        Pose2d target = APRIL_TAG_POS[targetId()];

        Translation2d coord = target.getTranslation().plus(transform.getTranslation().rotateBy(target.getRotation()));
        Rotation2d angle = target.getRotation().plus(transform.getRotation().unaryMinus());

        Pose2d pos = new Pose2d(coord,angle);
        //Pose2d pos = target.transformBy(transform);

        return pos.transformBy(camera.offset);
    }

    // Relative to Robot
    private Pose2d getPosVision() {
        if (!hasValidTarget())
            return new Pose2d();

        double distance = getVisionDistance() + HUB_RADIUS;
        double yaw = Units.degreesToRadians(targetYaw());
        Translation2d translation = new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));

        Transform2d transform = new Transform2d(translation,
                new Rotation2d(Units.degreesToRadians(swerve.getHeading())));
        Pose2d pos = HUB_POSITION.transformBy(transform.inverse());
        return pos.transformBy(camera.offset);

        // Pose2d pos = PhotonUtils.estimateFieldToRobot(getRelativeCamPos(gyroAngle),
        // HUB_POSITION, offset);
    }

    // Relative to Robot
    @Deprecated
    private Pose2d visionEstimatedPose() {
        if (!hasValidTarget())
            return new Pose2d();
        double distToHubCenter = getVisionDistance() + HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(swerve.getHeading() - targetYaw());
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()),
                -distToHubCenter * Math.sin(thetaHub.getRadians()))
                .plus(HUB_POSITION.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(swerve.getHeading()));
        pos.transformBy(camera.offset);

        return pos;
    }

    // Relative to Robot
    @Deprecated
    private Pose2d visionPos() {
        if (!hasValidTarget())
            return new Pose2d();
        double distToHubCenter = getVisionDistance() + HUB_RADIUS;
        double angle = Units.degreesToRadians(swerve.getHeading() - targetYaw() - 180);
        Translation2d fieldPos = new Translation2d(distToHubCenter * Math.cos(angle), distToHubCenter * Math.sin(angle))
                .plus(HUB_POSITION.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(swerve.getHeading()));
        pos.transformBy(camera.offset);

        return pos;
    }

    public void setPipeline(Pipeline pipeline) {
        sPipeline = pipeline;
        setPipelineIndex(pipeline.index);
    }

    public String get_name() {
        return camera.hostname;
    }

}
