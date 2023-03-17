package frc.team3128.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class CmdScoreAuto extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Swerve swerve;
    private Telescope telescope;
    private Manipulator manipulator;
    private NAR_XboxController controller;

    public CmdScoreAuto(boolean isReversed, ArmPosition position, int xpos) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        swerve = Swerve.getInstance();
        controller = RobotContainer.controller;
        addCommands(
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.parallel(
                new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], isReversed, VisionConstants.SCORES_GRID[xpos]),
                new InstantCommand(() -> pivot.startPID(position.pivotAngle), pivot)
            ),
            Commands.parallel(
                Commands.sequence(
                    new WaitUntilCommand(()-> Vision.MANUAL).raceWith(
                        new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? 0.35 : -0.35,0), 
                                                DriverStation.isAutonomous() ? 0 : -RobotContainer.controller.getRightX() * 1.5,true), swerve)
                        .withTimeout(0.75)),
                    new InstantCommand(()-> swerve.stop(), swerve)
                ),
                Commands.sequence(
                    new WaitUntilCommand(()-> pivot.atSetpoint()),
                    new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
                    new WaitUntilCommand(()-> telescope.atSetpoint())                    
                )
            ),
            new InstantCommand(() -> manipulator.outtake(), manipulator),
            new WaitCommand(0.125),
            new InstantCommand(() -> manipulator.stopRoller(), manipulator),
            new InstantCommand(() -> telescope.setSetpoint(ArmPosition.NEUTRAL.teleDist), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint()),
            new InstantCommand(()-> {telescope.disable(); telescope.engageBrake();}, telescope),
            new InstantCommand(()-> pivot.setSetpoint(Vision.GROUND_DIRECTION ? 15 : -15), pivot),
            new InstantCommand(() -> Vision.AUTO_ENABLED = DriverStation.isAutonomous())
        );
    }
}
