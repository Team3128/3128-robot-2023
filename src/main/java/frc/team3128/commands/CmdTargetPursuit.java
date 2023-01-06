package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.VisionConstants.*;

import frc.team3128.Constants.DriveConstants;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdTargetPursuit extends CommandBase {

    private Swerve drive;
    private Vision vision;
    private String camera;
    private Rotation2d camAngle;

    private PIDController distance, rotation;

    int targetCount, plateauCount;

    /**
     * Aligns robot to ball and pursuits autonomously 
     * @Requirements Drivetrain
     */
    public CmdTargetPursuit(String camera) {
        drive = Swerve.getInstance();
        vision = Vision.getInstance();
        camAngle = vision.camSpecs(camera).offset.getRotation();

        this.camera = camera;

        initControllers();
        addRequirements(drive);
    }

    public void initControllers() {
        distance = new PIDController(translationKP, translationKI, translationKD);
        distance.setSetpoint(2);
        distance.setTolerance(DRIVE_TOLERANCE);

        rotation = new PIDController(rotationKP, rotationKI, rotationKD);
        rotation.enableContinuousInput(-180, 180);
        rotation.setTolerance(TURN_TOLERANCE);
        rotation.setSetpoint(0);
    }

    @Override
    public void initialize() {
        distance.reset();
        rotation.reset();
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget(camera)) {
            Double dist = distance.calculate(vision.calculatedDistance(camera));
            Double spin = rotation.calculate(vision.getTx(camera));
            drive.drive(new Translation2d(dist,0).rotateBy(camAngle), Units.degreesToRadians(spin),false);
        }
        else {
            distance.reset();
            rotation.reset();
            drive.drive(new Translation2d(0,0), Units.degreesToRadians(120),false);
        }
        SmartDashboard.putBoolean("Rotation",rotation.atSetpoint());
        SmartDashboard.putBoolean("DistanceBoolean",distance.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        /*
        When the robot is moving very slowly + blind the robot has probably just intook (?)
        since it decelerated and there is no more target the limelight sees.
        */
        return distance.atSetpoint() && rotation.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

}