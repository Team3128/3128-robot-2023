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

public class CmdMovePickup extends CmdMove {

    public Pose2d position;
    private double PASS_LINE;
    private boolean isReversed;

    public CmdMovePickup(boolean isReversed, double maxSpeed, Pose2d position) {
        super(CmdMove.Type.LOADING, false, maxSpeed, position);
        this.position = position;
        this.isReversed = isReversed;
    }

    public CmdMovePickup(boolean isReversed, Pose2d position) {
        this(isReversed, SwerveConstants.maxSpeed, position);
    }

    @Override
    public void initialize() {
        var newPoses = new Pose2d[2];
        newPoses[0] = swerve.getPose().nearest(Arrays.asList(VisionConstants.RAMP_AVOID_LOADING));
        newPoses[1] = position;
        if (isReversed) {
            for (int i = 0; i < newPoses.length; i++) {
                var newRotation = new Rotation2d(MathUtil.inputModulus(poses[i].getRotation().getRadians() + Math.PI, -Math.PI, Math.PI));
                poses[i] = new Pose2d(poses[i].getTranslation(), newRotation);
            }
        }

        poses = newPoses;

        PASS_LINE = FieldConstants.chargingStationInnerX + SwerveConstants.robotLength/2 + 0.02;
        PASS_LINE = DriverStation.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - PASS_LINE : PASS_LINE;
        
        super.initialize();
    }

    @Override
    public void execute() {
        if ((pastX(PASS_LINE) && index == 0))
            nextPoint();
        super.execute();
    }
    
}
