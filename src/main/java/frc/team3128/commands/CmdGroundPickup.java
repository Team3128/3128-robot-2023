package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;


public class CmdGroundPickup extends SequentialCommandGroup{
    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();
    private Manipulator manipulator = Manipulator.getInstance();

    public CmdGroundPickup() {
        addCommands(
            new InstantCommand(()-> manipulator.intakeWide()),
            new CmdMoveArm(ArmPosition.GROUND_PICKUP),
            new InstantCommand(()->manipulator.setRollerPower(0.1)),
            new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }
    
}
