package frc.team3128.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.subsystems.Vision;

public class CmdMoveScore extends CmdMove {

    private double PASS_LINE;
    private boolean[] overrides;
    private ArrayList<Pose2d> avoidRamp;
    private Pose2d[][] positions;
    private int currSelectedGrid;
    private boolean isReversed;

    public CmdMoveScore(boolean[] overrides, boolean isReversed, double maxSpeed, Pose2d[]... positions) {
        super(CmdMove.Type.SCORE, true, positions[0]);
        this.positions = positions;
        this.overrides = overrides;
        this.isReversed = isReversed;
        avoidRamp = new ArrayList<Pose2d>();
    }

    public CmdMoveScore(boolean[] overrides, boolean isReversed, Pose2d[]... positions) {
        this(overrides, isReversed, SwerveConstants.maxSpeed, positions);
    }

    @Override
    public void initialize() {
        currSelectedGrid = Vision.SELECTED_GRID;
        int extraPoint = 0;
        var newPoses = new Pose2d[positions.length];
        Pose2d currentPose = swerve.getPose();
        if (overrides[currSelectedGrid]) {
            avoidRamp.clear();
            extraPoint = 1;
            newPoses = new Pose2d[positions.length + 1];
            for (Pose2d pose : VisionConstants.RAMP_AVOID_SCORE) {
                avoidRamp.add(FieldConstants.allianceFlip(pose));
            }
            newPoses[0] = currentPose.nearest(VisionConstants.RAMP_AVOID_SCORE);
        }
        for (int i = extraPoint; i < positions.length + extraPoint; i++) {
            newPoses[i] = positions[i - extraPoint][currSelectedGrid];
        }
        poses = newPoses;
        if (isReversed) {
            for (int i = 0; i < newPoses.length; i++) {
                var newRotation = new Rotation2d(MathUtil.inputModulus(poses[i].getRotation().getRadians() + Math.PI, -Math.PI, Math.PI));
                poses[i] = new Pose2d(poses[i].getTranslation(), newRotation);
            }
            for (int i = 0; i < avoidRamp.size(); i++) {
                var newRotation = new Rotation2d(MathUtil.inputModulus(avoidRamp.get(i).getRotation().getRadians() + Math.PI, -Math.PI, Math.PI));
                avoidRamp.set(i, new Pose2d(avoidRamp.get(i).getTranslation(), newRotation));
            }
        }
        PASS_LINE = FieldConstants.chargingStationOuterX - SwerveConstants.robotLength/2 - 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
        super.initialize();
    }

    @Override
    public void execute() {
        if (overrides[currSelectedGrid]) {
            if (pastX(PASS_LINE)) {
                nextPoint();
            }
            else if (!atLastPoint()) {
                setPoint(swerve.getPose().nearest(avoidRamp));
            }
        }
        super.execute();
    }
    
}
