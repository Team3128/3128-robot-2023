package frc.team3128.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.ManipulatorConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.constantsint.ConstantsInt.VisionConstants;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import frc.team3128.subsystems.Intake.IntakeState;

public class CmdManager {

    private static Pivot pivot = Pivot.getInstance();
    private static Telescope telescope = Telescope.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Manipulator manipulator = Manipulator.getInstance();
    private static Led led = Led.getInstance();
    private static NAR_XboxController controller = RobotContainer.controller;


    private CmdManager() {}

    public static CommandBase CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        return Commands.sequence(
            new InstantCommand(()-> Vision.position = position),
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.deadline(
                new WaitUntilCommand(()-> Vision.AUTO_ENABLED)
                //new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            Commands.parallel(
                //new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[xpos], true, VisionConstants.SCORES_GRID[xpos]),
                CmdPivot(position)
            ),
            vibrateController(),
            new InstantCommand(() -> Vision.AUTO_ENABLED = DriverStation.isAutonomous())
        );
    }

    public static CommandBase CmdShelfPickup(boolean cone) {
        return Commands.sequence(
            new InstantCommand(() -> led.setPickupColor(cone)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            CmdPivot(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE),
            CmdTele(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE),
            CmdManipGrab(cone),
            vibrateController(),
            new InstantCommand(() -> new InstantCommand(()-> Vision.AUTO_ENABLED = false))
        );
    }

    public static CommandBase CmdIntake() {
        return Commands.sequence(
            new InstantCommand(()-> Intake.objectPresent = false),
            CmdIntakeIntake(),
            CmdExtendIntake(IntakeState.DEPLOYED),
            new WaitUntilCommand(()-> intake.atSetpoint()).withTimeout(0.5),
            new WaitCommand(0.4),
            //new CmdCurrentCheck(intake.m_intakeRollers, IntakeConstants.CURRENT_THRESHOLD, IntakeConstants.ABSOLUTE_THRESHOLD),
            new WaitUntilCommand(()->intake.hasObjectPresent()),
            new InstantCommand(()-> Intake.objectPresent = true),
            new InstantCommand(()-> intake.stallPower(), intake),
            CmdExtendIntake(IntakeState.RETRACTED)
        );
    }
    
    public static CommandBase CmdManipGrab(boolean cone) {
        return Commands.sequence(
            CmdManipIntake(cone),
            new WaitCommand(0.4),
            //new CmdCurrentCheck(manipulator.m_roller, ManipulatorConstants.CURRENT_THRESHOLD, ManipulatorConstants.ABSOLUTE_THRESHOLD),
            new WaitUntilCommand(()-> manipulator.hasObjectPresent()),
            new WaitCommand(cone ? 0.1 : 0),
            new InstantCommand(()-> manipulator.stallPower(), manipulator)
        );
    }

    public static CommandBase CmdPivot(double angle) {
        return Commands.sequence(
            new InstantCommand(() -> pivot.startPID(angle), pivot),
            new WaitUntilCommand(()-> pivot.atSetpoint())
        );
    }

    public static CommandBase CmdPivot(ArmPosition position) {
        return CmdPivot(position.pivotAngle);
    }

    public static CommandBase CmdTele(double dist) {
        return Commands.sequence(
            new InstantCommand(() -> telescope.startPID(dist), telescope),
            new WaitUntilCommand(()-> telescope.atSetpoint())
        );
    }

    public static CommandBase CmdTele(ArmPosition position) {
        return CmdTele(position.teleDist);
    }

    public static CommandBase CmdExtendIntake(IntakeState state) {
        return new InstantCommand(()-> intake.startPID(state));
    }

    public static CommandBase CmdExtendRelease() {
        return new InstantCommand(() -> telescope.startPID(Vision.position), telescope);
    }

    public static CommandBase CmdManipIntake(boolean cone) {
        return new InstantCommand(()-> manipulator.intake(cone), manipulator);
    }

    public static CommandBase CmdManipOuttake() {
        return new InstantCommand(()-> manipulator.outtake(), manipulator);
    }

    public static CommandBase CmdStopManip() {
        return new InstantCommand(()-> manipulator.stopRoller(), manipulator);
    }

    public static CommandBase CmdIntakeIntake() {
        return new InstantCommand(()-> intake.intake(), intake);
    }

    public static CommandBase CmdIntakeOuttake() {
        return new InstantCommand(()-> intake.outtake(), intake);
    }

    public static CommandBase CmdStopIntake() {
        return new InstantCommand(()-> intake.stopRollers(), intake);
    }

    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }
}
