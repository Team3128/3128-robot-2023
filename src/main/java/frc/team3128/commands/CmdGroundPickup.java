package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Vision;

public class CmdGroundPickup extends SequentialCommandGroup {

    public CmdGroundPickup() {
        super(
            new CmdMoveArm(ArmPosition.GROUND_PICKUP),
            new CmdManipGrab(false),
            new CmdMoveArm(ArmPosition.NEUTRAL));
    }
}
