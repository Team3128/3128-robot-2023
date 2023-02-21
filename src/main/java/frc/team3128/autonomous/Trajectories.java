package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdHandoff;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdRetractIntake;
import frc.team3128.commands.CmdScore;
import frc.team3128.common.constantsint.ConstantsInt.VisionConstants;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Intake intake = Intake.getInstance();

    public static void initTrajectories() {
        final String[] trajectoryNames = {"r_top_1Cone", "r_top_1Cone+1Cube", "r_top_1Cone+1Cube+Climb",
                                            "b_top_1Cone", "b_top_1Cone+1Cube", "b_top_1Cone+1Cube+Climb",

                                            "r_mid_1Cone", "r_mid_1Cone+Climb",
                                            "b_mid_1Cone", "b_mid_1Cone+Climb",

                                            "r_bottom_1Cone", "r_bottom_1Cone+1Cube", "r_bottom_1Cone+1Cube+Climb",
                                            "b_bottom_1Cone", "b_bottom_1Cone+1Cube", "b_bottom_1Cone+1Cube+Climb", "TestAuto"
                                            };

        CommandEventMap.put("Score[1,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0])
                                                ));

        CommandEventMap.put("Score[2,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ArmPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[1])
                                                ));

        CommandEventMap.put("Score[2,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ArmPosition.MID_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[1])
                                                ));

        CommandEventMap.put("Score[8,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdScore(ArmPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[1])
                                                ));

        CommandEventMap.put("Score[8,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdScore(ArmPosition.MID_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[1])
                                                ));

        CommandEventMap.put("Score[9,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdScore(ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[2])
                                                ));

        CommandEventMap.put("Score[4,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 1),
                                                new CmdScore(ArmPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0])
                                                ));
        
        //StartScore

        CommandEventMap.put("StartScore[1,3]", new SequentialCommandGroup(
                                                new CmdMoveArm(ArmPosition.TOP_CONE),
                                                new InstantCommand(() -> Manipulator.getInstance().openClaw()),
                                                new WaitCommand(0.25),
                                                new InstantCommand(() -> Manipulator.getInstance().closeClaw()),
                                                new CmdMoveArm(ArmPosition.NEUTRAL)
                                                ));

        CommandEventMap.put("StartScore[2,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdMoveArm(ArmPosition.TOP_CUBE)
                                                
                                                ));

        CommandEventMap.put("StartScore[2,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdMoveArm(ArmPosition.MID_CONE)
                                                
                                                ));

        CommandEventMap.put("StartScore[8,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdMoveArm(ArmPosition.TOP_CUBE)
                                                ));

        CommandEventMap.put("StartScore[8,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdMoveArm(ArmPosition.MID_CUBE)
                                                ));

        CommandEventMap.put("StartScore[9,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 2),
                                                new CmdMoveArm(ArmPosition.TOP_CONE)
                                                ));

        CommandEventMap.put("StartScore[4,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 1),
                                                new CmdMoveArm(ArmPosition.TOP_CONE)
                                                ));

        
        CommandEventMap.put("IntakeCube", new SequentialCommandGroup(
            new CmdExtendIntake(),
            new WaitUntilCommand(()-> intake.checkObjectPresent()),
            //new WaitCommand(3),
            new CmdRetractIntake()
            //new CmdHandoff()
        )
            
        );

        CommandEventMap.put("Climb", new SequentialCommandGroup(
                                                new CmdInPlaceTurn(0),
                                                new CmdDriveUp(),
                                                new CmdGyroBalance()
                                                ));
        
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
    
}