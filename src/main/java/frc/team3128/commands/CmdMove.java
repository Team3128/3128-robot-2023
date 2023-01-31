package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.SwerveConstants.*;

import java.util.function.DoubleSupplier;

import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;

public class CmdMove extends CommandBase {

    public enum Type {
        SCORE(
            new double[][] {
                new double[] {RAMP_X_LEFT - trackWidth/2, RAMP_X_RIGHT + trackWidth/2},
                new double[] {LOADING_X_LEFT - trackWidth/2, LOADING_X_RIGHT}
             }),
        LOADING(
            new double[][] {
                new double[] {0,RAMP_X_RIGHT + trackWidth/2}
            }
        ),
        NONE(null);
        public double[][] constraints;

        private Type(double[][] constraints) {
            this.constraints = constraints;
        }
    }

    private static PIDController xController, yController;
    private static PIDController rController;
    private static DoubleSupplier xAxis, yAxis, throttle;
    private boolean xSetpoint, ySetpoint, rSetpoint, atDestination;
    protected Pose2d[] poses;
    private int index;
    private Type type;

    private boolean joystickOverride;

    private Swerve swerve;

    public CmdMove(Type type, boolean joystickOverride, Pose2d... poses) {
        this.poses = poses;
        this.type = type;

        this.joystickOverride = joystickOverride;

        swerve = Swerve.getInstance();
        index = 0;

        addRequirements(swerve);
    }

    public static void setController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier accel) {
        xAxis = x;
        yAxis = y;
        throttle = accel;
    }

    static {
        xController = new PIDController(translationKP, translationKI, translationKD);
        yController = new PIDController(translationKP, translationKI, translationKD);
        rController = new PIDController(rotationKP, rotationKI, rotationKD);
        rController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(DRIVE_TOLERANCE);
        yController.setTolerance(DRIVE_TOLERANCE);
        rController.setTolerance(Math.PI/120);

        NAR_Shuffleboard.addComplex("VisionPID","XCONTROLLER",xController,0,0);
        NAR_Shuffleboard.addComplex("VisionPID","YCONTROLLER",yController,1,0);
        NAR_Shuffleboard.addComplex("VisionPID","RCONTROLLER",rController,2,0);
    }

    @Override
    public void initialize() {
        index = 0;
        boolean keepSkipping = true;
        for (int i = 0; i < poses.length; i++) {
            poses[i] = allianceFlip(poses[i]);
            if (pastPoint(poses[i]) && !atLastPoint() && keepSkipping) {
                index += 1;
                continue;
            }
            keepSkipping = false;
        }
        xSetpoint = false;
        ySetpoint = false;
        rSetpoint = false;
        atDestination = false;

        Pose2d pos = swerve.getPose();

        xController.reset();
        yController.reset();
        rController.reset();

        xController.setSetpoint(poses[index].getX());
        yController.setSetpoint(poses[index].getY());
        rController.setSetpoint(poses[index].getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getPose(); 
        double xDistance = xController.calculate(pose.getX());
        double yDistance = yController.calculate(pose.getY()); 
        double rotation = rController.calculate(pose.getRotation().getRadians());

        xSetpoint = xController.atSetpoint();
        ySetpoint = yController.atSetpoint();
        rSetpoint = rController.atSetpoint();

        if (xSetpoint) {
            xDistance = 0;
        }
        if (ySetpoint || !canMove(pose.getX())) {
            yDistance = 0;
        }
        if (rSetpoint) {
            rotation = 0;
        }

        if (joystickOverride) {
            int team = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;
            if (Math.abs(xAxis.getAsDouble()) > 0.05) {
                yDistance = xAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * team;
            }
            if (Math.abs(yAxis.getAsDouble()) > 0.05) {
                xDistance = yAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * -team;
            }
        }

        swerve.drive(new Translation2d(xDistance, yDistance), rotation, true);

        if (xSetpoint && !atLastPoint()) {
            index += 1;
            xController.setSetpoint(poses[index].getX());
            yController.setSetpoint(poses[index].getY());
            rController.setSetpoint(poses[index].getRotation().getRadians());
            xSetpoint = false;
            ySetpoint = false;
            rSetpoint = false;
        }
        atDestination = (xSetpoint && ySetpoint && rSetpoint && atLastPoint());
    }

    @Override
    public boolean isFinished() {
        return atDestination;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CmdMove ended");
        swerve.stop();
    }

    public boolean canMove(double x) {
        if (type == Type.NONE) {
            return true;
        }
        double[][] constraints = type.constraints;
        for (int i = 0; i < constraints.length; i ++) {
            double left = DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - constraints[i][1] : constraints[i][0];
            double right = DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - constraints[i][0] : constraints[i][1];
            if (x >= left && x <= right){
                return false;
            }
        }
        return true;
    }

    private boolean pastPoint(Pose2d goalPos) {
        Pose2d currentPos = swerve.getPose();
        if (DriverStation.getAlliance() == Alliance.Red) {
            return (currentPos.getX() > goalPos.getX());
        }
        else {
            return (currentPos.getX() < goalPos.getX());
        }
    }

    private boolean atLastPoint() {
        return index == poses.length - 1;
    }

}