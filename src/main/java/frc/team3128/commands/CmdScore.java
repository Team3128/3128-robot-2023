package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], isReversed, VisionConstants.SCORES_GRID[xpos]).asProxy(),
                Commands.sequence(
                    new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
                    new InstantCommand(() -> pivot.startPID(isReversed ? -position.pivotAngle : position.pivotAngle), pivot)
                )
            ),
            Commands.parallel(
                Commands.sequence(
                    new WaitUntilCommand(()-> Vision.MANUAL).raceWith(
                        new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? 0.35 : -0.35,0),0,true), swerve)
                        .withTimeout(1)),
                    new InstantCommand(()-> swerve.stop(), swerve)
                ).asProxy(),
                Commands.sequence(
                    new WaitUntilCommand(()-> pivot.atSetpoint()),
                    new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
                    new WaitUntilCommand(()-> telescope.atSetpoint())                    
                )
            ),            
            new InstantCommand(() -> manipulator.outtake(position.cone), manipulator),
            new WaitCommand(0.125),
            new InstantCommand(() -> manipulator.stopRoller(), manipulator),
            // new InstantCommand(() -> pivot.startPID(position.pivotAngle)),
            // new WaitUntilCommand(() ->pivot.atSetpoint()),
            new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL, isReversed)),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())))
        );
    }
}
