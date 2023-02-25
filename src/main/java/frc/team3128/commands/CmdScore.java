package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class CmdScore extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Swerve swerve;
    private Telescope telescope;
    private Manipulator manipulator;

    public CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        swerve = Swerve.getInstance();

        addCommands(
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.parallel(
                //new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], isReversed, VisionConstants.SCORES_GRID[xpos]),
                Commands.sequence(
                    new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
                    new InstantCommand(() -> pivot.startPID(isReversed ? -position.pivotAngle : position.pivotAngle), pivot)
                )
            ),
            Commands.parallel(
                // Commands.sequence(
                //     new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? 0.25 : -0.25,0),0,true)).withTimeout(1),
                //     new InstantCommand(()-> swerve.stop())
                // ),
                Commands.sequence(
                    new WaitUntilCommand(()-> pivot.atSetpoint()),
                    new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
                    new WaitUntilCommand(()-> telescope.atSetpoint())
                )
            ),            
            new InstantCommand(() -> manipulator.outtake(), manipulator),
            new WaitCommand(0.125),
            // new InstantCommand(() -> pivot.startPID(position.pivotAngle + Math.copySign(10, position.pivotAngle))),
            new InstantCommand(() -> manipulator.neutralPos(), manipulator),
            // new InstantCommand(() -> telescope.startPID(ArmPosition.NEUTRAL.teleDist), telescope),
            // new WaitUntilCommand(() -> tlescope)
            new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL)),
            new WaitUntilCommand(()-> telescope.atSetpoint())
            // new CmdMoveArm(ArmPosition.NEUTRAL) // proxyschedulecmd this so you can start driving once it's going back in
        );
    }
}
