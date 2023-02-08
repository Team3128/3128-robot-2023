package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.subsystems.Vision;

public class CmdMoveLoading extends CmdMove {

    public Pose2d[] positions;
    private int currentSelectedGrid; 
    private double PASS_LINE;

    public CmdMoveLoading(Pose2d... positions) {
        super(CmdMove.Type.LOADING, true, positions);
        this.positions = positions;
    }

    @Override
    public void initialize() {
        currentSelectedGrid = Vision.SELECTED_GRID;
        var newPoses = new Pose2d[positions.length + 1];
        newPoses[0] = currentSelectedGrid == 0 ? VisionConstants.RAMP_AVOID_LOADING[0] : VisionConstants.RAMP_AVOID_LOADING[1];
        for (int i = 1; i < positions.length; i ++) {
            newPoses[i] = positions[i - 1];
        }
        super.initialize();
        PASS_LINE = FieldConstants.chargingStationInnerX + SwerveConstants.trackWidth/2 + 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
    }

    @Override
    public void execute() {
        if (currentSelectedGrid == 0 && pastX(PASS_LINE) && !atLastPoint()) {
            nextPoint();
        }
        super.execute();
    }
    
}
