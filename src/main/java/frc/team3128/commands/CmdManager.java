package frc.team3128.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
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
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.common.constantsint.ConstantsInt.VisionConstants;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Leds;

public class CmdManager {

    private static Pivot pivot = Pivot.getInstance();
    private static Telescope telescope = Telescope.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Manipulator manipulator = Manipulator.getInstance();
    private static Leds leds = Leds.getInstance();
    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean isChute = true;


    private CmdManager() {}

    public static CommandBase CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        return sequence(
            new InstantCommand(()-> Vision.position = position),
            new InstantCommand(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            // new CmdTrajectory(xpos, position == ArmPosition.LOW_FLOOR),
            either(none(), CmdPivot(position), ()-> position == ArmPosition.LOW_FLOOR),
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            vibrateController()
        );
    }

    public static CommandBase CmdPickup(ArmPosition position, boolean runImmediately) {
        return sequence(
            runOnce(() -> leds.setPivotLeds(position.cone ? Colors.CONE : Colors.CUBE)),
            new InstantCommand(()-> Vision.AUTO_ENABLED = runImmediately),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            runOnce(()-> Vision.AUTO_ENABLED = false),
            runOnce(() -> leds.setPivotLeds(position.cone ? Colors.CONE : Colors.CUBE)),
            new CmdMoveArm(position),
            CmdManipGrab(position.cone),
            new InstantCommand(() -> leds.setPivotLeds(Colors.HOLDING)),
            new WaitCommand(0.333),
            new InstantCommand(() -> leds.setPivotLeds(Colors.DEFAULT)),
            new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }

    public static CommandBase CmdPickup(ArmPosition position) {
        return CmdPickup(position, false);
    }

    public static CommandBase CmdIntake() {
        return sequence(
            new InstantCommand(()-> Intake.objectPresent = false),
            CmdIntakeIntake(),
            CmdExtendIntake(IntakeState.DEPLOYED),
            new WaitUntilCommand(()-> intake.atSetpoint()).withTimeout(0.5),
            new WaitCommand(0.4),
            //new CmdCurrentCheck(intake.m_intakeRollers, IntakeConstants.CURRENT_THRESHOLD, IntakeConstants.ABSOLUTE_THRESHOLD),
            new WaitUntilCommand(()->intake.hasObjectPresent()),
            new InstantCommand(()-> Intake.objectPresent = true),
            new InstantCommand(()-> intake.stallPower(), intake),
            new InstantCommand(() -> leds.setPivotLeds(Colors.HOLDING)),
            CmdExtendIntake(IntakeState.RETRACTED),
            new InstantCommand(() -> leds.setPivotLeds(Colors.DEFAULT))
        );
    }
    
    public static CommandBase CmdManipGrab(boolean cone) {
        return sequence(
            CmdManipIntake(cone),
            new WaitCommand(0.4),
            //new CmdCurrentCheck(manipulator.m_roller, ManipulatorConstants.CURRENT_THRESHOLD, ManipulatorConstants.ABSOLUTE_THRESHOLD),
            new WaitUntilCommand(()-> manipulator.hasObjectPresent()),
            new WaitCommand(cone ? 0.1 : 0),
            new InstantCommand(()-> manipulator.stallPower(), manipulator)
        );
    }

    public static CommandBase CmdPivot(double angle) {
        return sequence(
            new InstantCommand(() -> pivot.startPID(angle), pivot),
            new WaitUntilCommand(()-> pivot.atSetpoint())
        );
    }

    public static CommandBase CmdPivot(ArmPosition position) {
        return CmdPivot(position.pivotAngle);
    }

    public static CommandBase CmdTele(double dist) {
        return sequence(
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
