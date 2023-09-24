package frc.team3128.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.team3128.commands.CmdMoveArm;

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
import frc.team3128.common.constantsint.ConstantsInt.VisionConstants;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;
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
    private static Swerve swerve = Swerve.getInstance();
    private static Led led = Led.getInstance();
    private static NAR_XboxController controller = RobotContainer.controller;


    private CmdManager() {}

    public static CommandBase CmdScore(boolean isReversed, ArmPosition position, int xpos) {
        return Commands.sequence(
            run(()-> Vision.position = position),
            run(() -> NarwhalDashboard.setGridCell(xpos,position.height)),
            run(()-> Vision.AUTO_ENABLED = DriverStation.isAutonomous()),
            Commands.deadline(
                waitUntil(()-> Vision.AUTO_ENABLED)
                //new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true)
            ),
            Commands.parallel(
                new CmdTrajectory(xpos),
                CmdPivot(position)
            ),
            vibrateController()
        );
    }

    public static CommandBase CmdShelfPickup(boolean cone) {
        return Commands.sequence(
            run(() -> led.setPickupColor(cone)),
            run(()-> Vision.AUTO_ENABLED = false),
            waitUntil(()-> Vision.AUTO_ENABLED),
            CmdPivot(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE),
            CmdTele(cone ? ArmPosition.HP_SHELF_CONE : ArmPosition.HP_SHELF_CUBE),
            CmdManipGrab(cone),
            vibrateController(),
            run(() -> run(()-> Vision.AUTO_ENABLED = false))
        );
    }

    public static CommandBase CmdIntake() {
        return Commands.sequence(
            run(()-> Intake.objectPresent = false),
            CmdIntakeIntake(),
            CmdExtendIntake(IntakeState.DEPLOYED),
            waitUntil(()-> intake.atSetpoint()).withTimeout(0.5),
            waitSeconds(0.4),
            //new CmdCurrentCheck(intake.m_intakeRollers, IntakeConstants.CURRENT_THRESHOLD, IntakeConstants.ABSOLUTE_THRESHOLD),
            waitUntil(()->intake.hasObjectPresent()),
            run(()-> Intake.objectPresent = true),
            run(()-> intake.stallPower(), intake),
            CmdExtendIntake(IntakeState.RETRACTED)
        );
    }
    public static CommandBase CmdOuttake() {
        return Commands.sequence(
            CmdManipOuttake(),
            waitSeconds(0.35),
            CmdStopManip(),
            CmdSwerveThrottle(0.8),
            new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }
    
    public static CommandBase CmdManipGrab(boolean cone) {
        return Commands.sequence(
            CmdManipIntake(cone),
            waitSeconds(0.4),
            //new CmdCurrentCheck(manipulator.m_roller, ManipulatorConstants.CURRENT_THRESHOLD, ManipulatorConstants.ABSOLUTE_THRESHOLD),
            waitUntil(()-> manipulator.hasObjectPresent()),
            waitSeconds(cone ? 0.1 : 0),
            run(()-> manipulator.stallPower(), manipulator)
        );
    }

    
    //swerve commands
    public static CommandBase CmdSwerveThrottle(double throttle) {
        return run(()-> Swerve.throttle = throttle);
    }
    public static CommandBase CmdSwerveStop() {
        return run(()-> swerve.stop(),swerve);
    }
    public static CommandBase CmdSwerveResetEncoders() {
        return run(()-> swerve.resetEncoders());
    }
    public static CommandBase CmdSwerveZeroGyro() {
        return run(()->swerve.zeroGyro());
    }

    //vision commands


    //controller commands
    public static CommandBase CmdStartVibrateController() {
        return run(()-> controller.startVibrate());
    }
    public static CommandBase CmdStopVibrateController() {
        return run(()-> controller.stopVibrate());
    }
    
    //system check commands
    public static CommandBase CmdSystemCheckRepeat() {
        return run(()-> CmdSystemCheckFancy.repeat = true);
    }

    //intake commands
    public static CommandBase CmdExtendIntake(IntakeState state) {
        return run(()-> intake.startPID(state));
    }
    public static CommandBase CmdRunIntake(double power) {
        return run(()-> intake.moveIntake(power), intake);
    }
    public static CommandBase CmdIntakeHasObject() {
        return run(()-> intake.set(intake.hasObjectPresent() ? 0.1 : 0), intake);
    }

    //pivot commands
    public static CommandBase CmdPivot(ArmPosition position) {
        return CmdPivot(position.pivotAngle);
    }

    public static CommandBase CmdRunPivot(double power) {
        return run(()-> pivot.setPower(power), pivot);
    }

    public static CommandBase CmdStopPivot() {
        return run(()-> pivot.setPower(0.0), pivot);
    }

    public static CommandBase CmdResetPivot() {
        return run(()-> pivot.resetPivot(), pivot);
    }

    //telescope commands
    public static CommandBase CmdTeleRetract() {
        return run(()-> telescope.retract(), telescope);
    }

    public static CommandBase CmdTeleExtend() {
        return run(()-> telescope.extend(), telescope);
    }

    public static CommandBase CmdTeleStop() {
        return run(()-> telescope.stopTele(), telescope);
    }

    public static CommandBase CmdTeleReleaseBrake() {
        return run(()-> telescope.releaseBrake(), telescope);
    }

    public static CommandBase CmdTeleZeroEncoder() {
        return run(()-> telescope.zeroEncoder(), telescope);
    }

    //manip commands
    public static CommandBase CmdManipStallPower(){
        return run(()-> manipulator.stallPower(), manipulator);
    }

    public static CommandBase CmdPivot(double angle) {
        return Commands.sequence(
            run(() -> pivot.startPID(angle), pivot),
            waitUntil(()-> pivot.atSetpoint())
        );
    }

    public static CommandBase CmdTele(double dist) {
        return Commands.sequence(
            run(() -> telescope.startPID(dist), telescope),
            waitUntil(()-> telescope.atSetpoint())
        );
    }

    public static CommandBase CmdTele(ArmPosition position) {
        return CmdTele(position.teleDist);
    }


    public static CommandBase CmdExtendRelease() {
        return run(() -> telescope.startPID(Vision.position), telescope);
    }

    public static CommandBase CmdManipIntake(boolean cone) {
        return run(()-> manipulator.intake(cone), manipulator);
    }

    public static CommandBase CmdManipOuttake() {
        return run(()-> manipulator.outtake(), manipulator);
    }

    public static CommandBase CmdStopManip() {
        return run(()-> manipulator.stopRoller(), manipulator);
    }

    public static CommandBase CmdIntakeIntake() {
        return run(()-> intake.intake(), intake);
    }

    public static CommandBase CmdIntakeOuttake() {
        return run(()-> intake.outtake(), intake);
    }

    public static CommandBase CmdStopIntake() {
        return run(()-> intake.stopRollers(), intake);
    }

    public static CommandBase vibrateController() {
        return new ScheduleCommand(waitSeconds(0.5).deadlineWith(startEnd(() -> RobotContainer.controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }
}
