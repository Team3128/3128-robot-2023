package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ScoringPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class CmdScore extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Swerve swerve;
    private Telescope telescope;
    private Manipulator manipulator;

    public CmdScore(ScoringPosition position, boolean[] overrides, Pose2d[]... positions) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        swerve = Swerve.getInstance();

        addCommands(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new CmdMoveScore(overrides, positions),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(() -> pivot.startPID(position.pivotAngle), pivot),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? 0.25 : -0.25,0),0,true)).withTimeout(1),
            new InstantCommand(()-> swerve.stop()),
            
            new WaitUntilCommand(()-> pivot.atSetpoint()),
            
            new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new InstantCommand(() -> manipulator.openClaw(), manipulator),
            new WaitCommand(0.25),
            //new InstantCommand(() -> pivot.startPID(position.pivotAngle + Math.copySign(10, position.pivotAngle))),
            new CmdRetractArm()
            // new WaitCommand(2),
            // new InstantCommand(() -> telescope.startPID(ScoringPosition.NEUTRAL))
        );
    }
}
