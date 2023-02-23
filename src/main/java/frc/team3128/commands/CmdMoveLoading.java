package frc.team3128.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.subsystems.Vision;

public class CmdMoveLoading extends CmdMove {

    public Pose2d[][] positions;
    private int currentSelectedGrid; 
    private double PASS_LINE;

    public CmdMoveLoading(Pose2d[]... positions) {
        super(CmdMove.Type.LOADING, true, positions[0]);
        this.positions = positions;
    }

    @Override
    public void initialize() {
        currentSelectedGrid = Vision.SELECTED_GRID;
        var newPoses = new Pose2d[positions.length + 1];
        newPoses[0] = swerve.getPose().nearest(Arrays.asList(VisionConstants.RAMP_AVOID_LOADING));
        for (int i = 1; i < positions.length + 1; i ++) {
            newPoses[i] = positions[i - 1][currentSelectedGrid];
        }
        PASS_LINE = FieldConstants.chargingStationInnerX + SwerveConstants.robotLength/2 + 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
        super.initialize();
    }

    @Override
    public void execute() {
        if (pastX(PASS_LINE) && !atLastPoint()) {
            nextPoint();
        }
        super.execute();
    }
    
}
