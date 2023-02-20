package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;


public class CmdGroundPickup extends SequentialCommandGroup{
    private Pivot pivot = Pivot.getInstance();
    private Telescope telescope = Telescope.getInstance();
    private Manipulator manipulator = Manipulator.getInstance();

    public CmdGroundPickup() {
        addCommands(
            new InstantCommand(()-> manipulator.openClaw()),
            new InstantCommand(()-> pivot.startPID(ArmConstants.IntakePosition.GROUND_PICKUP.pivotAngle)),
            new InstantCommand(()-> telescope.startPID(ArmConstants.IntakePosition.GROUND_PICKUP.teleDist)),
            new CmdRetractArm()
        );
    }
    
}
