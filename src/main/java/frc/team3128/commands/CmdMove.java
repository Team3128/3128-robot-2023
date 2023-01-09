package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.TX_THRESHOLD;

import java.util.function.DoubleSupplier;

import frc.team3128.Constants.FieldConstants;
import frc.team3128.subsystems.Swerve;

public class CmdMove extends CommandBase {

    private PIDController xController, yController, rController;
    private DoubleSupplier xAxis, yAxis, throttle;
    private boolean xSetpoint, ySetpoint, rSetpoint;

    private boolean joystickOverride = false;

    private Swerve swerve;

    public CmdMove(Pose2d pose, DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier throttle) {
        this(pose);

        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.throttle = throttle;
        joystickOverride = true;
    }

    public CmdMove(Pose2d pose) {
        initControllers();

        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        rController.setSetpoint(pose.getRotation().getRadians());

        swerve = Swerve.getInstance();

        addRequirements(swerve);
    }

    private void initControllers() {
        xController = new PIDController(translationKP, translationKI, translationKD);
        yController = new PIDController(translationKP, translationKI, translationKD);
        rController = new PIDController(rotationKP, rotationKI, rotationKD);
        rController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0);
        yController.setTolerance(0);
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
        double xDistance = xSetpoint ? xController.calculate(pose.getX()) : 0;
        double yDistance = (ySetpoint && (pose.getX() > FieldConstants.RAMP_X_RIGHT || pose.getX() < FieldConstants.RAMP_X_LEFT)) ? yController.calculate(pose.getY()) : 0; 
        double rotation = rSetpoint ? rController.calculate(pose.getRotation().getRadians()) : 0;

        xSetpoint = xController.atSetpoint();
        ySetpoint = yController.atSetpoint();
        rSetpoint = rController.atSetpoint();

        if (joystickOverride) {
            if (Math.abs(xAxis.getAsDouble()) > 0.05) {
                yDistance = -xAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed;
            }
            if (Math.abs(yAxis.getAsDouble()) > 0.05) {
                xDistance = yAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed;
            }
        }

        swerve.drive(new Translation2d(xDistance, yDistance), rotation, true);
    }

    @Override
    public boolean isFinished() {
        return (xSetpoint && ySetpoint && rSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }


}
