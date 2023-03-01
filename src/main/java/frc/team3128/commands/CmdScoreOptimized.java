package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdScoreOptimized extends SequentialCommandGroup {
    public CmdScoreOptimized(ArmPosition position, int xPos) {
        super(
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xPos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(() -> {
                double rotation = MathUtil.inputModulus(Swerve.getInstance().getGyroRotation2d().getDegrees(), -180, 180);
                boolean isReversed = DriverStation.getAlliance() == Alliance.Red ? Math.abs(rotation) > 90 : Math.abs(rotation) < 90;
                if (Vision.FIXED_DIRECTION != null)
                    isReversed = Vision.FIXED_DIRECTION;
                new CmdScore(isReversed, position, xPos).schedule();
            })
        );
    }
}
