package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.commands.CmdBangBangBalance;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdIntake;
import frc.team3128.commands.CmdBalance;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.commands.CmdMoveLoading;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdScore;
import frc.team3128.commands.CmdScoreAuto;
import frc.team3128.commands.CmdMove.Type;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;
import frc.team3128.subsystems.Intake.IntakeState;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Manipulator manipulator = Manipulator.getInstance();

    private static Swerve swerve = Swerve.getInstance();

    // private static Intake intake = Intake.getInstance();

    public static void initTrajectories() {
        final String[] trajectoryNames = {"r_top_1Cone", "r_top_1Cone+1Cube", "r_top_1Cone+1Cube+Climb",
                                            "b_top_1Cone", "b_top_1Cone+1Cube", "b_top_1Cone+1Cube+Climb",

                                            "r_mid_1Cone", "r_mid_1Cone+Climb",
                                            "b_mid_1Cone", "b_mid_1Cone+Climb",

                                            "r_bottom_1Cone", "r_bottom_1Cone+1Cube", "r_bottom_1Cone+1Cube+Climb",
                                            "b_bottom_1Cone", "b_bottom_1Cone+1Cube", "b_bottom_1Cone+1Cube+Climb"
                                            };

        CommandEventMap.put("Score[2,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(true, ArmPosition.TOP_CUBE, 1)
                                                ));

        CommandEventMap.put("Score[2,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(true, ArmPosition.MID_CUBE, 1)
                                                ));

        CommandEventMap.put("Score[8,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdScore(true, ArmPosition.TOP_CUBE, 1)
                                                ));

        CommandEventMap.put("Score[8,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdScore(true, ArmPosition.MID_CUBE, 1)
                                                ));

        //StartScore

        CommandEventMap.put("ScoreConeHigh", new SequentialCommandGroup(
                                                new CmdMoveArm(ArmPosition.TOP_CONE),
                                                new InstantCommand(() -> manipulator.outtake()),
                                                new WaitCommand(0.125),
                                                new InstantCommand(() -> manipulator.stopRoller()),
                                                new ScheduleCommand(new CmdMoveArm(ArmPosition.NEUTRAL))
                                                ));
        
        // CommandEventMap.put("IntakeCube", new CmdGroundPickup());

        CommandEventMap.put("Climb", new SequentialCommandGroup(
                                                // new CmdInPlaceTurn(0),
                                                new CmdDriveUp(),
                                                new CmdBangBangBalance()
                                                ));
        
        CommandEventMap.put("ClimbPoseBlue", new CmdMove(Type.SCORE, false, new Pose2d(5.8,2.7,Rotation2d.fromDegrees(0))));
        
        CommandEventMap.put("ClimbPoseRed", new CmdMove(Type.SCORE, false, new Pose2d(10.7,2.7,Rotation2d.fromDegrees(0))));
        
        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
        }

        builder = new SwerveAutoBuilder(
            Swerve.getInstance()::getPose,
            Swerve.getInstance()::resetOdometry,
            swerveKinematics,
            new PIDConstants(1,0,0),
            new PIDConstants(1,0,0),
            Swerve.getInstance()::setModuleStates,
            CommandEventMap,
            Swerve.getInstance()
        );
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
            new PathConstraints(maxSpeed, maxAcceleration), 
            new PathPoint(start.getTranslation(), start.getRotation()), 
            new PathPoint(end.getTranslation(), end.getRotation())
            );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }

    // public static CommandBase manipPoint(Pose2d pose, boolean cone) {
    //     return Commands.sequence(
    //         new InstantCommand(()->Vision.AUTO_ENABLED = true),
    //         new CmdMove(Type.LOADING, false, pose),
    //         Commands.race(
    //             new CmdGroundPickup(),
    //             Commands.sequence(
    //                 new WaitCommand(0.5),
    //                 new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.5 : 0.5,0), 0,true), swerve)
    //                     .withTimeout(1.25)
    //             )
    //         ),
    //         new InstantCommand(()-> swerve.stop(), swerve),
    //         new InstantCommand(() -> manipulator.stopRoller()),
    //         new CmdMoveArm(ArmPosition.NEUTRAL)
    //     );
    // }

    public static CommandBase intakePoint(Pose2d pose) {
        return Commands.sequence(
            new InstantCommand(()->Vision.AUTO_ENABLED = true),
            Commands.race(
                new CmdIntake(),
                // new CmdGroundPickup(cone),
                Commands.sequence(
                    new CmdMove(Type.LOADING, false, pose),
                    new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.5 : 0.5,0), 0,true), swerve)
                        .withTimeout(1.25)
                )
            ),
            new InstantCommand(()->Intake.getInstance().startPID(IntakeState.RETRACTED), Intake.getInstance()),
            new InstantCommand(()-> swerve.stop(), swerve)
        );
    }

    public static CommandBase intakePointRamp(Pose2d pose) {
        return Commands.sequence(
            new InstantCommand(()->Vision.AUTO_ENABLED = true),
            Commands.race(
                new CmdIntake(),
                // new CmdGroundPickup(cone),
                Commands.sequence(
                    new CmdMoveLoading(false, new Pose2d[] {
                        pose,
                        pose,
                        pose
                    }),
                    new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.5 : 0.5,0), 0,true), swerve)
                        .withTimeout(1.25)
                )
            ),
            new InstantCommand(()->Intake.getInstance().startPID(IntakeState.RETRACTED), Intake.getInstance()),
            new InstantCommand(()-> swerve.stop(), swerve)
        );
    }

    public static CommandBase movePoint(Pose2d pose) {
        return Commands.sequence(
            new InstantCommand(()->Vision.AUTO_ENABLED = true),
            new CmdMove(Type.LOADING, false, pose)
        ); 
    }

    public static CommandBase intakePointSpecial(Pose2d pose) {
        return Commands.sequence(
            new InstantCommand(()->Vision.AUTO_ENABLED = true),
            new CmdMove(Type.NONE, false, pose),
            Commands.race(
                //new CmdIntake(),
                // new CmdGroundPickup(cone),
                Commands.sequence(
                    new WaitCommand(0.5),
                    new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.5 : 0.5,0), 0,true), swerve)
                        .withTimeout(1.25)
                )
            ),
            new InstantCommand(()-> swerve.stop(), swerve)
        );
    }

    public static CommandBase scoringPoint(int grid, int node, boolean reversed, ArmPosition position) {
        return Commands.sequence(
            new InstantCommand(()-> Vision.SELECTED_GRID = grid),
            new CmdScoreAuto(reversed, position, node)
        );
    }

    public static CommandBase scoreIntake(int grid, int node) {
        return Commands.sequence(
            new InstantCommand(()-> Vision.SELECTED_GRID = grid),
            new CmdMoveScore(VisionConstants.RAMP_OVERRIDE[node], true, VisionConstants.SCORES_GRID[node]),
            new InstantCommand(()-> Intake.getInstance().setReverse()),
            new WaitCommand(0.125),
            new InstantCommand(()->Intake.getInstance().stop())
        );
    }

    public static CommandBase startScoringPoint(boolean reversed, ArmPosition position) {
        return Commands.sequence(
            //new InstantCommand(() -> swerve.resetOdometry(FieldConstants.allianceFlip(AutoConstants.STARTING_POINTS[grid * 3 + node]))),
            new CmdMoveArm(position).withTimeout(3),
            new InstantCommand(()-> manipulator.outtake(), manipulator),
            new WaitCommand(0.125),
            new InstantCommand(()-> manipulator.stopRoller(), manipulator),
            new CmdMoveArm(ArmPosition.NEUTRAL)
        );
    }

    public static CommandBase resetOdometry(boolean front) {
        return Commands.sequence(
            new InstantCommand(()-> Vision.AUTO_ENABLED = true),
            new InstantCommand(()-> swerve.zeroGyro((DriverStation.getAlliance() == Alliance.Red && front) || (DriverStation.getAlliance() == Alliance.Blue && !front) ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.7 : 0.7,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(front ? VisionConstants.FRONT : VisionConstants.BACK).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> Vision.getInstance().visionReset())
        );
    }

    public static CommandBase climbPoint(boolean inside, boolean bottom) {
        return Commands.sequence(
            // new CmdMoveArm(ArmPosition.NEUTRAL, false),
            new InstantCommand(()-> Vision.getInstance().disableVision()),
            new CmdMove(Type.NONE, false, inside ? AutoConstants.ClimbSetupInside : (bottom ? AutoConstants.ClimbSetupOutsideBot : AutoConstants.ClimbSetupOutsideTop)),
            new InstantCommand(()-> Vision.getInstance().enableVision()),
            Commands.deadline(Commands.sequence(new WaitUntilCommand(()-> Math.abs(swerve.getRoll()) > 6), new CmdBangBangBalance()), new CmdBalance()), 
                                            //new RunCommand(()-> swerve.drive(new Translation2d(CmdBalance.DIRECTION ? -0.25 : 0.25,0),0,true)).withTimeout(0.5), 
                                            new RunCommand(()->Swerve.getInstance().xlock(), Swerve.getInstance())
            // new CmdMoveArm(90, 11.5, false)
        );
    }
    
}