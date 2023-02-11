package frc.team3128.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
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

    public CmdMoveScore(boolean[] overrides, Pose2d[]... positions) {
        super(CmdMove.Type.SCORE, true, positions[0]);
        this.positions = positions;
        this.overrides = overrides;
        avoidRamp = new ArrayList<Pose2d>();
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
        for (int i = 0; i < newPoses.length; i++) {
            System.out.println(newPoses[i]);
        }
        PASS_LINE = FieldConstants.chargingStationOuterX - SwerveConstants.trackWidth/2 - 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
        super.initialize();
    }

    @Override
    public void execute() {
        if (overrides[currSelectedGrid]) {
            if (pastX(PASS_LINE)) {
                nextPoint();
            }
            else {
                setPoint(swerve.getPose().nearest(avoidRamp));
            }
        }
        super.execute();
    }
    
}
