package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.VisionConstants.*;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdTargetPursuit extends CommandBase {

    private Swerve drive;
    private Vision vision;
    private String camera;
    private Rotation2d camAngle;

    private PIDController distance, rotation;
    private boolean atDistance, atRotation;

    private double dist;

    /**
     * Aligns robot to ball and pursuits autonomously 
     * @Requirements Drivetrain
     */
    public CmdTargetPursuit(String camera, double distance) {
        drive = Swerve.getInstance();
        vision = Vision.getInstance();
        camAngle = vision.camSpecs(camera).offset.getRotation();

        this.camera = camera;
        this.dist = distance;

        initControllers();
        addRequirements(drive);
    }

    public void initControllers() {
        distance = new PIDController(distanceKP, distanceKI, distanceKD);
        distance.setSetpoint(dist);
        distance.setTolerance(DRIVE_TOLERANCE);

        rotation = new PIDController(alignKP, alignKI, alignKD);
        rotation.enableContinuousInput(-180, 180);
        rotation.setTolerance(TX_THRESHOLD);
        rotation.setSetpoint(0);
    }

    @Override
    public void initialize() {
        atDistance = false;
        atRotation = false;
        distance.reset();
        rotation.reset();
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget(camera)) {
            Double dist = !atDistance ? distance.calculate(vision.calculateDistance(camera)) : 0;

            Double spin = !atRotation ? rotation.calculate(vision.getTX(camera)) : 0;
            drive.drive(new Translation2d(-dist,0).rotateBy(camAngle), -spin,false);
            atDistance = distance.atSetpoint();
            atRotation = rotation.atSetpoint();
        }
        else {
            distance.reset();
            rotation.reset();
            drive.drive(new Translation2d(0,0), Units.degreesToRadians(120),false);
        }
    }

    @Override
    public boolean isFinished() {
        return atDistance && atRotation;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

}