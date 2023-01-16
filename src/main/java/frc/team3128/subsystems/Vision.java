package frc.team3128.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.HashMap;

import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Vision extends SubsystemBase{
    private static Vision instance;

    private HashMap<String,NAR_Camera> cameras;

    public static synchronized Vision getInstance(){
        if(instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        Swerve swerve = Swerve.getInstance();
        NAR_Camera.setGyro(swerve::getYaw);
        NAR_Camera.setOdometry((pose,time) -> swerve.odometry.addVisionMeasurement(pose,time));
        NAR_Camera.setAprilTags(APRIL_TAG_POS);
        NAR_Camera.setVisionTarget(FieldConstants.HUB_POSITION);
        NAR_Camera.multipleTargets = true;
        cameras = new HashMap<String,NAR_Camera>();
        cameras.put(SHOOTER.hostname, new NAR_Camera(SHOOTER));
    }

    public Pose2d targetPos(String name, Pose2d robotPos) {
        return cameras.get(name).getTargetPos(robotPos);
    }

    public Pose2d robotPos(String name) {
        return cameras.get(name).getPos();
    }

    public double calculateDistance(String name) {
        return cameras.get(name).getDistance();
    }

    public double getTx(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetYaw();
    }

    public double getTy(String name) {
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
        SmartDashboard.putBoolean("HasTarget", cameras.get(SHOOTER.hostname).hasValidTarget());
        SmartDashboard.putNumber("Distance", calculateDistance(SHOOTER.hostname));
        SmartDashboard.putString("EstimatedPose",cameras.get(SHOOTER.hostname).getPos().toString());
        // if(cameras.get(Camera.SHOOTER.hostname).hasValidTarget()) {
        //     Swerve.getInstance().resetOdometry(cameras.get(Camera.SHOOTER.hostname).getPos());
        // }
        //SmartDashboard.putString("Aflack",cameras.get(Camera.SHOOTER.hostname).getPos().relativeTo(FieldConstants.HUB_POSITION).toString());
        SmartDashboard.putNumber("Geico",cameras.get(SHOOTER.hostname).targetYaw());
        SmartDashboard.putString("TARGETPOS",cameras.get(SHOOTER.hostname).getTargetPos(Swerve.getInstance().getPose()).toString());
        SmartDashboard.putString("RAWTARGET",cameras.get(SHOOTER.hostname).getTarget().toString());
        SmartDashboard.putNumber("TARGETGUITY",cameras.get(SHOOTER.hostname).targetAmbiguity());
        SmartDashboard.putString("PROCESSED TARGET", cameras.get(SHOOTER.hostname).getProcessedTarget().toString());
        SmartDashboard.putString("SUNSHINE", cameras.get(SHOOTER.hostname).getRawTarget().toString());
    }

    public void initShuffleboard() {
        NAR_Camera cam = cameras.get(SHOOTER.hostname);
        NAR_Shuffleboard.addData("Vision","HasTarget", ()->cam.hasValidTarget(), 1, 1);
        NAR_Shuffleboard.addData("Vision","Distance",()->cam.getDistance(),2,1);
        NAR_Shuffleboard.addData("Vision","RawTarget",()->cam.getTarget().toString(),1,2,3,1);
        NAR_Shuffleboard.addData("Vision", "Processed Target",()->cam.getProcessedTarget().toString(),1,3,3,1);
        NAR_Shuffleboard.addData("Vision","EstimatedPose", ()-> cam.getPos(),1,4,3,1);
    }
}