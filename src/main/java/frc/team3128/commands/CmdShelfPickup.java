package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;


    public CmdShelfPickup (Pose2d... poses) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        addCommands(
            new InstantCommand(() -> manipulator.openClaw(), manipulator),
            new InstantCommand(() -> pivot.startPID(ArmConstants.IntakePosition.HP_SHELF.pivotAngle), pivot),
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            //new CmdMoveLoading(poses),
            new InstantCommand(() -> telescope.startPID(ArmConstants.IntakePosition.HP_SHELF.teleDist), telescope)
            
            // new InstantCommand(() -> pivot.startPID(angle), pivot),
            // new WaitCommand(3),
            // new InstantCommand(() -> telescope.startPID(dist), telescope)

        );
    }
}
