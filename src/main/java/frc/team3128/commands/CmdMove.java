package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.TX_THRESHOLD;

import java.util.function.DoubleSupplier;

import static frc.team3128.Constants.FieldConstants.*;
import frc.team3128.subsystems.Swerve;

public class CmdMove extends CommandBase {

    private PIDController xController, yController, rController;
    private static DoubleSupplier xAxis, yAxis, throttle;
    private double[] xConstraints, yConstraints;
    private boolean xSetpoint, ySetpoint, rSetpoint;

    private boolean joystickOverride;

    private Swerve swerve;

    public CmdMove(Pose2d pose, double[] xConstraints, double[] yConstraints, boolean joystickOverride) {
        this.xConstraints = xConstraints;
        this.yConstraints = yConstraints;

        initControllers();

        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        rController.setSetpoint(pose.getRotation().getRadians());

        this.joystickOverride = joystickOverride;

        swerve = Swerve.getInstance();

        addRequirements(swerve);
    }

    public CmdMove(Pose2d pose, boolean joystickOverride) {
        this(pose, new double[] {RAMP_X_LEFT, RAMP_X_RIGHT}, new double[] {0,0}, joystickOverride);
    }

    public static void setController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier accel) {
        xAxis = x;
        yAxis = y;
        throttle = accel;
    }

    private void initControllers() {
        xController = new PIDController(translationKP, translationKI, translationKD);
        yController = new PIDController(translationKP, translationKI, translationKD);
        rController = new PIDController(rotationKP, rotationKI, rotationKD);
        rController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(DRIVE_TOLERANCE);
        yController.setTolerance(DRIVE_TOLERANCE);
        rController.setTolerance(TX_THRESHOLD);
    }

    @Override
    public void initialize() {
        xSetpoint = false;
        ySetpoint = false;
        rSetpoint = false;

        xController.reset();
        yController.reset();
        rController.reset();
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getPose(); 
        double xDistance = (xSetpoint && (!inYConstraint() || inXConstraint())) ? xController.calculate(pose.getX()) : 0;
        double yDistance = (ySetpoint && (!inXConstraint() || inYConstraint())) ? yController.calculate(pose.getY()) : 0; 
        double rotation = rSetpoint ? rController.calculate(pose.getRotation().getRadians()) : 0;

        xSetpoint = xController.atSetpoint();
        ySetpoint = yController.atSetpoint();
        rSetpoint = rController.atSetpoint();

        if (joystickOverride) {
            int team = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;
            if (Math.abs(xAxis.getAsDouble()) > 0.05) {
                yDistance = xAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * team;
            }
            if (Math.abs(yAxis.getAsDouble()) > 0.05) {
                xDistance = yAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * -team;
            }
        }

        swerve.drive(new Translation2d(0, yDistance), 0, true);
    }

    @Override
    public boolean isFinished() {
        return (xSetpoint && ySetpoint && rSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    public boolean inXConstraint() {
        double x = swerve.getPose().getX();
        return (x >= xConstraints[0] && x <= xConstraints[1]);
    }

    public boolean inYConstraint() {
        double y = swerve.getPose().getY();
        return (y >= yConstraints[0] && y <= yConstraints[1]);
    }

}