package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdPickupOptimized extends SequentialCommandGroup {

    public CmdPickupOptimized(boolean cone) {
        super(
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(() -> {
                double rotation = MathUtil.inputModulus(Swerve.getInstance().getGyroRotation2d().getDegrees(), -180, 180);
                boolean isReversed = Math.abs(rotation) < 90;
                if (cone) Vision.FIXED_DIRECTION = isReversed;
                else Vision.FIXED_DIRECTION = null;
                new CmdShelfPickup(cone, isReversed).schedule();
            })
        );
    }

}
