package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Vision;

public class CmdGroundPickup extends InstantCommand {

    // Vision.GROUND_DIRECTION = false will pick up from battery side which will score cone on front side
    // Vision.GROUND_DIRECTION = true will pick up from front and score cone on battery side
    public CmdGroundPickup(boolean cone) {
        super(()-> Commands.sequence(
            new CmdMoveArm(cone ? ArmPosition.GROUND_PICKUP_CONE : ArmPosition.GROUND_PICKUP_CUBE, !Vision.GROUND_DIRECTION),
            new CmdManipGrab(cone),
            new InstantCommand(
                ()-> {
                    if (cone) Vision.FIXED_DIRECTION = Vision.GROUND_DIRECTION;
                    else Vision.FIXED_DIRECTION = null;
                }
            ),
            new CmdMoveArm(ArmPosition.NEUTRAL, false)).schedule()
        );
    }
}
