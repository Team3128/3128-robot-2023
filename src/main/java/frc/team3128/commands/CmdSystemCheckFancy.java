package frc.team3128.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.swerve.SwerveModule;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Intake.IntakeState;

import static frc.team3128.Constants.ManipulatorConstants.*;
import static frc.team3128.commands.CmdManager.*;

import java.util.Arrays;

import static frc.team3128.Constants.IntakeConstants.*;

public class CmdSystemCheckFancy extends CommandBase {
    public static double systemCheck = 0;
    public static boolean repeat = true;

    private Swerve swerve;
    private Telescope tele;
    private Pivot pivot;
    private Manipulator manip;
    private Intake intake;

    private double driveVelocity = 1;

    private static boolean swerveSystemCheck, armSystemCheck, manipulatorSystemCheck, intakeSystemCheck = false;

    public CmdSystemCheckFancy() {
        swerve = Swerve.getInstance();
        tele = Telescope.getInstance();
        pivot = Pivot.getInstance();
        manip = Manipulator.getInstance();
        intake = Intake.getInstance();
    }

    @Override
    public void initialize() {
        systemCheck = 0;
        repeat = true;
        swerveSystemCheck = false;
        armSystemCheck = false;
        manipulatorSystemCheck = false;
        intakeSystemCheck = false;
    }

    @Override
    public void execute() {
        if (!repeat) return;
        repeat = false;
        if (systemCheck == 1) {
            swerveSystemCheck = false;
            swerveCheck(driveVelocity);
        }
        else if (systemCheck == 2) {
            CommandBase armTest = Commands.sequence(
                new InstantCommand(()-> armSystemCheck = false),
                new CmdMoveArm(ArmPosition.NEUTRAL),
                new WaitCommand(1),
                new CmdMoveArm(ArmPosition.TOP_CONE.pivotAngle, 25),
                new WaitCommand(1),
                new CmdMoveArm(90, 11.5),
                new InstantCommand(()-> armSystemCheck = true)
            );
            armTest.schedule();
        }
        else if (systemCheck == 3) {
            CommandBase armCheck = Commands.sequence(
                new InstantCommand(()-> manipulatorSystemCheck = false),
                new CmdMoveArm(90,11.5),
                CmdManipGrab(true),
                new WaitCommand(2),
                new StartEndCommand(()-> manip.outtake(), ()-> manip.stopRoller(), manip).withTimeout(2),
                new WaitCommand(1),
                CmdManipGrab(false),
                new WaitCommand(2),
                new StartEndCommand(()-> manip.outtake(), ()-> manip.stopRoller(), manip).withTimeout(2),
                new CmdMoveArm(ArmPosition.NEUTRAL),
                new InstantCommand(()-> manipulatorSystemCheck = true)
            );
            armCheck.schedule();
        }
        else if (systemCheck == 4) {
            CommandBase intakeCheck = Commands.sequence(
                new InstantCommand(()-> intakeSystemCheck = false),
                CmdIntake(),
                new WaitCommand(1),
                new StartEndCommand(()-> intake.outtake(), ()-> intake.stopRollers(), intake).withTimeout(1),
                new InstantCommand(()-> intakeSystemCheck = true)
            );
            intakeCheck.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return systemCheck > 4;
    }

    public void swerveCheck(double velocity) {
        for (int i = 0; i < 8; i++) {
            double angle = i * 45;
            SwerveModuleState desiredTestState = new SwerveModuleState(velocity * (angle > 180 ? -1 : 1),
                    Rotation2d.fromDegrees(angle));
            SwerveModuleState[] desiredTestStates = new SwerveModuleState[4];
            Arrays.fill(desiredTestStates, desiredTestState);
            swerve.setModuleStates(desiredTestStates);
            Timer.delay(1);
            System.out.println("Angle: " + i);
            for (SwerveModule module : swerve.modules) {
                System.out.println(
                        "Module " + module.moduleNumber + ": " + swerve.compare(module.getState(), desiredTestState));
            }
        }
        swerve.stop();
        swerveSystemCheck = true;
    }

    public static void initShuffleboard() {
        NAR_Shuffleboard.addData("System Check", "Count", ()-> systemCheck, 1,0);
        NAR_Shuffleboard.addData("System Check", "Swerve", () -> swerveSystemCheck, 0, 0);
        NAR_Shuffleboard.addData("System Check", "Arm", () -> armSystemCheck, 0, 1);
        NAR_Shuffleboard.addData("System Check", "Intake", () -> intakeSystemCheck, 0, 2);
        NAR_Shuffleboard.addData("System Check", "Manipulator", () -> manipulatorSystemCheck, 0, 3);
    }
}
