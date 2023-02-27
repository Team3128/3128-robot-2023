package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants.ArmPosition;

public class CmdGroundPickup extends SequentialCommandGroup{

    public CmdGroundPickup(boolean cone) {
        addCommands(
            new CmdMoveArm(ArmPosition.GROUND_PICKUP, false),
            new CmdManipGrab(cone),
            new CmdMoveArm(ArmPosition.NEUTRAL, false)
        );
    }
    
}
