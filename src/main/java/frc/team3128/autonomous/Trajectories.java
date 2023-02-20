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

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.ArmConstants.ScoringPosition;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdScore;
import frc.team3128.common.constantsint.ConstantsInt.VisionConstants;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap;

    public static void initTrajectories() {
        final String[] trajectoryNames = {"r_top_1Cone", "r_top_1Cone+1Cube", "r_top_1Cone+1Cube+Climb",
                                            "b_top_1Cone", "b_top_1Cone+1Cube", "b_top_1Cone+1Cube+Climb",

                                            "r_mid_1Cone", "r_mid_1Cone+Climb",
                                            "b_mid_1Cone", "b_mid_1Cone+Climb",

                                            "r_bottom_1Cone", "r_bottom_1Cone+1Cube", "r_bottom_1Cone+1Cube+Climb",
                                            "b_bottom_1Cone", "b_bottom_1Cone+1Cube", "b_bottom_1Cone+1Cube+Climb",
                                            };

        CommandEventMap.put("Score[1,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ScoringPosition.TOP_CONE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[0])
                                                ));

        CommandEventMap.put("Score[2,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[3])
                                                ));

        CommandEventMap.put("Score[2,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 0),
                                                new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[4])
                                                ));

        CommandEventMap.put("Score[8,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 3),
                                                new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[3])
                                                ));

        CommandEventMap.put("Score[8,2]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 3),
                                                new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[4])
                                                ));

        CommandEventMap.put("Score[9,3]", new SequentialCommandGroup(
                                                new InstantCommand(()-> Vision.SELECTED_GRID = 3),
                                                new CmdScore(ScoringPosition.TOP_CUBE, VisionConstants.RAMP_OVERRIDE[0], VisionConstants.SCORES_GRID[6])
                                                ));

        
        CommandEventMap.put("IntakeCube", null);

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
            new PIDConstants(translationKP,translationKI,translationKD),
            new PIDConstants(rotationKP,rotationKI,rotationKD),
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