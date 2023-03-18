package frc.team3128.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

import java.util.Arrays;

import static frc.team3128.Constants.IntakeConstants.*;


public class CmdSystemCheck extends SequentialCommandGroup{
    private Swerve swerve;
    private Telescope tele;
    private Pivot pivot;
    private Manipulator manip;
    private Intake intake;

    private double driveVelocity = 1;

    private double checkCounter = 0;

    boolean moduleFunction[] = new boolean[4];

    private static boolean swerveSystemCheck, armSystemCheck, manipulatorSystemCheck, intakeSystemCheck = false;

    public CmdSystemCheck() {
        swerve = Swerve.getInstance();
        tele = Telescope.getInstance();
        pivot = Pivot.getInstance();
        manip = Manipulator.getInstance();
        intake = Intake.getInstance();

        RobotContainer.systemCheck = 0;
        
        addCommands(
            new WaitUntilCommand(()-> RobotContainer.systemCheck == 1),
            new InstantCommand(()-> swerveCheck(driveVelocity)),
            new WaitUntilCommand(()-> RobotContainer.systemCheck == 2),
            new CmdMoveArm(ArmPosition.NEUTRAL).withTimeout(3),
            new CmdMoveArm(ArmPosition.TOP_CONE.pivotAngle, 25).withTimeout(3),
            new CmdMoveArm(ArmPosition.NEUTRAL).withTimeout(3),
            new InstantCommand(()-> armSystemCheck = true),
            new WaitUntilCommand(()-> RobotContainer.systemCheck == 3),
            new CmdIntake(),
            new WaitCommand(1),
            new InstantCommand(()-> intake.outake()),
            new WaitUntilCommand(()-> RobotContainer.systemCheck == 4),
            new CmdManipGrab(true),
            new WaitCommand(1),
            new InstantCommand(()-> manip.reverse()),
            new CmdManipGrab(false),
            new WaitCommand(1),
            new InstantCommand(()-> manip.reverse()),
            new InstantCommand(()-> manipulatorSystemCheck = true)

        );

        addRequirements(swerve, tele, pivot, manip, intake);
    }

    public void swerveCheck(double velocity) {
        for(int i = 0; i < 8; i++){
            double angle  = i*45;
            SwerveModuleState desiredTestState = new SwerveModuleState(velocity * (angle > 180 ? -1 : 1), new Rotation2d(angle));
            SwerveModuleState[] desiredTestStates = new SwerveModuleState[4];
            Arrays.fill(desiredTestStates, desiredTestState);
            swerve.setModuleStates(desiredTestStates);
            Timer.delay(0.2);
            System.out.println("Angle: " + i);
            for(SwerveModule module: swerve.modules){
                System.out.println("Module " + module.moduleNumber + ": " + swerve.compare(module.getState(), desiredTestState));
            }
        }
    }

    public static void initShuffleboard() {
        NAR_Shuffleboard.addData("System Check", "Swerve", ()-> swerveSystemCheck, 0, 0);
        NAR_Shuffleboard.addData("System Check", "Arm", ()-> armSystemCheck, 0, 1);
        NAR_Shuffleboard.addData("System Check", "Intake", ()-> intakeSystemCheck, 0, 2);
        NAR_Shuffleboard.addData("System Check", "Manipulator", ()-> manipulatorSystemCheck, 0, 3);
    }
}

