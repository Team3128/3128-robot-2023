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

public class CmdScore extends SequentialCommandGroup {
    
    private Pivot pivot;
    private Swerve swerve;
    private Telescope telescope;
    private Manipulator manipulator;
    private NAR_XboxController controller;

    public CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        swerve = Swerve.getInstance();
        controller = RobotContainer.controller;
        addCommands(
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.deadline(
                new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
                new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            // new InstantCommand(() -> {if (DriverStation.isAutonomous()) Vision.AUTO_ENABLED = true;}),
            Commands.parallel(
                // new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], isReversed, VisionConstants.SCORES_GRID[xpos]),
                // Commands.sequence(
                //     new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
                new InstantCommand(() -> pivot.startPID(-position.pivotAngle), pivot)
            ),
            Commands.deadline(
                Commands.sequence(
                    new WaitUntilCommand(()-> pivot.atSetpoint()),
                    new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
                    new WaitUntilCommand(()-> telescope.atSetpoint())                    
                ),
                new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            // Commands.parallel(
            //     Commands.sequence(
            //         new WaitUntilCommand(()-> Vision.MANUAL).raceWith(
            //             new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? 0.35 : -0.35,0), 
            //                                     DriverStation.isAutonomous() ? 0 : -RobotContainer.controller.getRightX() * 1.5,true), swerve)
            //             .withTimeout(0.75)),
            //         new InstantCommand(()-> swerve.stop(), swerve)
            //     ),
            //     Commands.sequence(
            //         new WaitUntilCommand(()-> pivot.atSetpoint()),
            //         new InstantCommand(() -> telescope.startPID(position.teleDist), telescope),
            //         new WaitUntilCommand(()-> telescope.atSetpoint())                    
            //     )
            // ),
            // new InstantCommand(() -> manipulator.outtake(position.cone), manipulator),
            // new WaitCommand(0.125),
            // new InstantCommand(() -> manipulator.stopRoller(), manipulator),
            // // new InstantCommand(() -> pivot.startPID(position.pivotAngle)),
            // // new WaitUntilCommand(() ->pivot.atSetpoint()),
            // new InstantCommand(() -> Manipulator.objectPresent = false),
            // new InstantCommand(() -> telescope.setSetpoint(ArmPosition.NEUTRAL.teleDist), telescope),
            // new WaitUntilCommand(()-> telescope.atSetpoint()),
            // new InstantCommand(()-> {telescope.disable(); telescope.engageBrake();}, telescope),
            // new InstantCommand(()-> pivot.setSetpoint(Vision.GROUND_DIRECTION ? 15 : -15), pivot),
            // //new CmdMoveArm(ArmPosition.NEUTRAL, isReversed),
            new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate()))),
            new InstantCommand(() -> Vision.AUTO_ENABLED = DriverStation.isAutonomous())
        );
    }
}
