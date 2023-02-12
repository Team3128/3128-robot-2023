package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ScoringPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CmdScore extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;

    public CmdScore(ScoringPosition position, boolean[] overrides, Pose2d[]... positions) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();

        addCommands(
            new InstantCommand(() -> pivot.startPID(position.pivotAngle), pivot),
            // new CmdMoveScore(overrides, positions),
            
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            
            new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new InstantCommand(() -> manipulator.openClaw(), manipulator),
            new WaitCommand(0.25),
            new InstantCommand(() -> pivot.startPID(position.pivotAngle + Math.copySign(10, position.pivotAngle))),
            new CmdRetractArm(),
            new InstantCommand(() -> manipulator.closeClaw(), manipulator)
            // new WaitCommand(2)
            // new InstantCommand(() -> telescope.startPID(ScoringPosition.NEUTRAL.))
        );
    }
}
