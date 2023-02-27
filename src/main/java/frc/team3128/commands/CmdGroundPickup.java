package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Vision;

public class CmdGroundPickup extends SequentialCommandGroup{

    // isReversed = false will pick up from battery side which will score cone on front side
    // isReversed = true will pick up from front and score cone on battery side
    public CmdGroundPickup(boolean cone) {
        boolean isReversed = Vision.GROUND_DIRECTION;
        addCommands(
            new CmdMoveArm(cone ? ArmPosition.GROUND_PICKUP_CONE : ArmPosition.GROUND_PICKUP_CUBE, !isReversed),
            new CmdManipGrab(cone),
            new InstantCommand(
                ()-> {
                    if (cone) Vision.FIXED_DIRECTION = isReversed;
                    else Vision.FIXED_DIRECTION = null;
                }
            ),
            new CmdMoveArm(ArmPosition.NEUTRAL, false)
        );
    }
}
