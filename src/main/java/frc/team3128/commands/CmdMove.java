package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.SwerveConstants.*;

import java.util.function.DoubleSupplier;

import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdMove extends CommandBase {
    
    public enum Type {
        SCORE(
            new double[][] {
                new double[] {chargingStationInnerX - robotLength/2.0 - 0.01, chargingStationOuterX + robotLength/2.0 + 0.01},
                new double[] {LOADING_X_LEFT - robotLength/2.0, LOADING_X_RIGHT}
            },
            new double[] {   //Rectangular Constraint
                chargingStationOuterX + robotLength/2.0 - 0.1,
                FIELD_X_LENGTH/2,
                chargingStationLeftY + robotLength/2.0 + 0.1,
                chargingStationRightY - robotLength/2.0 - 0.1
            },
            5.6,
            true
        ),
        LOADING(
            new double[][] {
                new double[] {chargingStationInnerX - robotLength/2.0,chargingStationOuterX + robotLength/2.0}
            },
            new double[] {   //Rectangular Constraint
                0, //chargingStationInnerX - robotLength,
                chargingStationInnerX - robotLength/2.0,
                chargingStationLeftY + robotLength/2.0 + 0.1,
                chargingStationRightY - robotLength/2.0 - 0.1
            },
            2.3,
            false
        ),
        NONE(null,null, null, false);
        public double[][] xConstraints;
        public double[] boxConstraints;
        public Double deadLine;
        public boolean movingLeft; //Relative to Blue

        private Type(double[][] xConstraints, double[] boxConstraints, Double deadLine, boolean movingLeft) {
            this.xConstraints = xConstraints;
            this.boxConstraints = boxConstraints;
            this.deadLine = deadLine;
            this.movingLeft = movingLeft;
        }
    }

    private static PIDController xController, yController;
    private static PIDController xDeadController;
    private static PIDController rController;
    private static DoubleSupplier xAxis, yAxis, rAxis, throttle;
    private boolean xSetpoint, ySetpoint, rSetpoint, atDestination;
    private boolean inXDead;
    protected Pose2d[] poses;
    protected int index;
    private Type type;
    private double maxSpeed;

    private boolean joystickOverride;

    protected Swerve swerve;

    public CmdMove(Type type, boolean joystickOverride, double maxSpeed, Pose2d... poses) {
        this.poses = poses;
        this.type = type;
        this.maxSpeed = maxSpeed;
        this.joystickOverride = joystickOverride;
        atDestination = false;

        swerve = Swerve.getInstance();
        index = 0;

        addRequirements(swerve);
    }

    public CmdMove(Type type, boolean joystickOverride, Pose2d... poses) {
        this(type, joystickOverride, SwerveConstants.maxSpeed, poses);
    }

    public static void setController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, DoubleSupplier accel) {
        xAxis = x;
        yAxis = y;
        rAxis = r;
        throttle = accel;
    }

    static {
        xController = new PIDController(translationKP, translationKI, translationKD);
        yController = new PIDController(translationKP, translationKI, translationKD);
        xDeadController = new PIDController(translationKP, translationKI, translationKD);
        rController = new PIDController(rotationKP, rotationKI, rotationKD);
        rController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(DRIVE_TOLERANCE);
        xDeadController.setTolerance(DRIVE_TOLERANCE);
        yController.setTolerance(DRIVE_TOLERANCE);
        rController.setTolerance(Math.PI/60);

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
            if (pastX(poses[i].getX()) && !atLastPoint() && keepSkipping && type != Type.NONE) {
                index += 1;
                continue;
            }
            keepSkipping = false;
        }

        xSetpoint = false;
        ySetpoint = false;
        rSetpoint = false;
        atDestination = false;

        inXDead = false;
        //Vision.AUTO_ENABLED = false;

        xController.reset();
        yController.reset();
        rController.reset();

        xController.setSetpoint(poses[index].getX());
        yController.setSetpoint(poses[index].getY());
        rController.setSetpoint(poses[index].getRotation().getRadians());
        if (type != Type.NONE)
            xDeadController.setSetpoint(xDeadLine(type.deadLine));
    }

    @Override
    public void execute() {
        Pose2d pose = swerve.getPose();
        Rotation2d Rotation = swerve.getGyroRotation2d();
        double xDistance = xController.calculate(pose.getX());
        //xDistance = xDistance + Math.signum(xDistance) * VisionConstants.AUTO_FF;
        double yDistance = yController.calculate(pose.getY()); 
        //yDistance = yDistance + Math.signum(yDistance) * VisionConstants.AUTO_FF;
        double rotation = rController.calculate(Rotation.getRadians());

        xSetpoint = xController.atSetpoint();
        ySetpoint = yController.atSetpoint();
        rSetpoint = rController.atSetpoint();

        inXDead = !canMoveX(pose);
        if (xSetpoint || inXDead) xDistance = 0;
        if (ySetpoint || !canMoveY(pose)) yDistance = 0;
        if (rSetpoint) rotation = 0;

        //if ((xDistance == 0) && !xSetpoint) yDistance = Math.copySign(maxSpeed, yDistance);
        //Try adding logic so you can't move past the value until you clear the barrier
        if (inXDead)
            xDistance = xDeadController.calculate(pose.getX());
        if (nearBump())
            xDistance = Math.min(xDistance, bumpSpeed);
        
        if (Math.abs(yDistance) > maxSpeed)
            xDistance = Math.min(Math.abs(xDistance), maxSpeed) * Math.signum(xDistance);
        if (Math.abs(xDistance) > maxSpeed && Math.abs(yDistance) < maxSpeed)
            xDistance = (Math.sqrt(Math.pow(maxSpeed,2) - Math.pow(yDistance,2))) * Math.signum(xDistance);
        if (Math.abs(xDistance) < maxSpeed && yDistance > maxSpeed)
            yDistance = (Math.sqrt(Math.pow(maxSpeed,2) - Math.pow(xDistance,2))) * Math.signum(yDistance);

        if (!Vision.AUTO_ENABLED) {
            xDistance = 0;
            yDistance = 0;
            rotation = 0;
        }

        if (joystickOverride) {
            int team = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;
            if (Math.abs(xAxis.getAsDouble()) > 0.05 || Math.abs(yAxis.getAsDouble()) > 0.05 || Math.abs(rAxis.getAsDouble()) > 0.05) {
                yDistance = xAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * team;
                xDistance = yAxis.getAsDouble() * throttle.getAsDouble() * maxSpeed * -team;
                rotation = -rAxis.getAsDouble() * maxAngularVelocity;
            }
        }

        Translation2d translation = new Translation2d(xDistance, yDistance);
        if (translation.getNorm() > maxSpeed) {
            Rotation2d driveAngle = translation.getAngle();
            translation = new Translation2d(maxSpeed, driveAngle);
        }

        swerve.drive(translation, rotation, true);

        if (xSetpoint && ySetpoint && !atLastPoint()) {
            nextPoint();
            return;
        }

        atDestination = (xSetpoint && ySetpoint && atLastPoint());
    }

    private boolean canMoveX(Pose2d pose) {
        if (type == Type.NONE) return true;
        if (type == Type.SCORE && pose.getY() >= 5.1) return false;
        double[] xConstraints = new double[] {type.boxConstraints[0], type.boxConstraints[1]};
        double[] yConstraints = new double[] {type.boxConstraints[2], type.boxConstraints[3]};
        return (!inXConstraints(xConstraints, pose.getX()) || !inYConstraints(yConstraints, pose.getY()));
    }

    private boolean canMoveY(Pose2d pose) {
        if (type == Type.NONE) return true;
        // if (pose.getY() < chargingStationRightY - robotLength/2 - 0.05 && yController.getSetpoint() < chargingStationRightY - robotLength/2 - 0.05) return true;
        // if (pose.getY() > chargingStationLeftY + robotLength/2 + 0.05 && yController.getSetpoint() > chargingStationLeftY + robotLength/2 + 0.05) return true;
        double[][] constraints = type.xConstraints;
        for (int i = 0; i < constraints.length; i ++) {
            if (inXConstraints(constraints[i], pose.getX())){
                return false;
            }
        }
        return true;
    }

    private boolean inXConstraints(double[] constraints, double xPos) {
        double left = DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - constraints[1] : constraints[0];
        double right = DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - constraints[0] : constraints[1];
        return (xPos >= left && xPos <= right);
    }

    private double xDeadLine(double deadline) {
        if (DriverStation.getAlliance() == Alliance.Red)
            deadline = FIELD_X_LENGTH - deadline;
        return deadline;
    }

    private boolean inYConstraints(double[] constraints, double yPos) {
        double top = constraints[0];
        double bottom = constraints[1];
        return (yPos >= bottom && yPos <= top);
    }

    private boolean nearBump() {
        Pose2d pose = swerve.getPose();
        return inXConstraints(new double[]{cableBumpInnerX - robotLength, cableBumpOuterX + robotLength}, pose.getX()) &&
            inYConstraints(new double[]{chargingStationRightY, 0}, FIELD_X_LENGTH);
    }

    protected void nextPoint() {
        if (atLastPoint()) return;
        index += 1;
        setPoint(poses[index]);
    }

    protected void setPoint(Pose2d pose) {
        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        rController.setSetpoint(pose.getRotation().getRadians());
        xSetpoint = false;
        ySetpoint = false;
        rSetpoint = false;
    }

    protected boolean pastX(double x) {
        Pose2d currentPos = swerve.getPose();
        boolean movingLeft = (type.movingLeft && DriverStation.getAlliance() == Alliance.Blue) || (!type.movingLeft && DriverStation.getAlliance() == Alliance.Red);
        if (movingLeft) return (currentPos.getX() < x);
        return (currentPos.getX() > x);
    }

    protected boolean atLastPoint() {
        return index == poses.length - 1;
    }

    @Override
    public boolean isFinished() {
        return atDestination || Vision.MANUAL;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
