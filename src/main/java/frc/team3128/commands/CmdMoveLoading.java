package frc.team3128.commands;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private boolean isReversed;

    public CmdMoveLoading(boolean isReversed, double maxSpeed, Pose2d[]... positions) {
        super(CmdMove.Type.LOADING, true, maxSpeed, positions[0]);
        this.positions = positions;
        this.isReversed = isReversed;
    }

    public CmdMoveLoading(boolean isReversed, Pose2d[]... positions) {
        this(isReversed, SwerveConstants.maxSpeed, positions);
    }

    @Override
    public void initialize() {
        currentSelectedGrid = Vision.SELECTED_GRID;
        var newPoses = new Pose2d[positions.length + 2];
        newPoses[0] = swerve.getPose().nearest(Arrays.asList(VisionConstants.RAMP_AVOID_LOADING));
        newPoses[1] = VisionConstants.HPWall_Loading;
        for (int i = 2; i < newPoses.length; i ++) {
            newPoses[i] = positions[i - 1][currentSelectedGrid];
        }

        if (isReversed) {
            for (int i = 0; i < newPoses.length; i++) {
                var newRotation = new Rotation2d(MathUtil.inputModulus(poses[i].getRotation().getRadians() + Math.PI, -Math.PI, Math.PI));
                poses[i] = new Pose2d(poses[i].getTranslation(), newRotation);
            }
        }

        PASS_LINE = FieldConstants.chargingStationInnerX + SwerveConstants.robotLength/2 + 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
        
        super.initialize();
    }

    @Override
    public void execute() {
        if ((pastX(PASS_LINE) && index == 0) || (swerve.getPose().getY() > VisionConstants.WALL_PASS && index == 1))
            nextPoint();
        
        super.execute();
    }
    
}
