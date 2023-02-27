package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Vision;

public class CmdGroundPickup extends SequentialCommandGroup{

    public CmdGroundPickup(boolean cone) {
        addCommands(
            new CmdMoveArm(cone ? ArmPosition.GROUND_PICKUP_CONE : ArmPosition.GROUND_PICKUP_CUBE, false),
            new CmdManipGrab(cone),
            new CmdMoveArm(ArmPosition.NEUTRAL, false),
            new InstantCommand(
                ()-> {
                    Vision.FIXED_DIRECTION = null;
                }
            )
        );
    }
}
