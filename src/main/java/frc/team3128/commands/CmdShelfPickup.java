package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.ArmConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;

public class CmdShelfPickup extends SequentialCommandGroup{
    
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;


    public CmdShelfPickup (Pose2d... poses) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        addCommands(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new CmdMoveLoading(poses),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(() -> manipulator.openClaw(), manipulator),
            new CmdMoveArm(ArmPosition.HP_SHELF),
            new InstantCommand(() -> manipulator.closeClaw()),
            new WaitCommand(0.25),
            new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }
}
