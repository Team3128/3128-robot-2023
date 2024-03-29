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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.ArmConstants;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.PivotConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.CmdBangBangBalance;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdManager;

import static frc.team3128.commands.CmdManager.*;

import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.commands.CmdBalance;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.commands.CmdMovePickup;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdScoreAuto;
import frc.team3128.commands.CmdMove.Type;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
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

    public static double autoSpeed = SwerveConstants.maxSpeed;


    public static void initTrajectories() {
        final String[] trajectoryNames = {
                                        //Blue Autos
                                            //Cable
                                            "b_cable_1Cone+1Cube","b_cable_1Cone+2Cube", "b_cable_1Cone+2Cube+Climb",
                                            //Mid
                                            "b_mid_1Cone+Climb","b_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "b_hp_1Cone+1Cube","b_cable_1Cone+2Cube",
                                            
                                        //Red Autos
                                            //Cable
                                            "r_cable_1Cone+1Cube","r_cable_1Cone+2Cube",
                                            //Mid
                                            "r_mid_1Cone+Climb","r_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "r_hp_1Cone+1Cube","r_cable_1Cone+2Cube",
                                        };

        CommandEventMap.put("Rotate180", new CmdInPlaceTurn(180));
        CommandEventMap.put("ScoreConeHigh", new SequentialCommandGroup(
                                                new CmdMoveArm(ArmPosition.TOP_CONE),
                                                new InstantCommand(() -> manipulator.outtake()),
                                                new WaitCommand(.5),
                                                new InstantCommand(() -> manipulator.stopRoller()),
                                                new CmdMoveArm(ArmPosition.NEUTRAL)
                                                ));

        CommandEventMap.put("ScoreConeLow", Commands.sequence(
            CmdPivot(30),
            CmdPivot(ArmPosition.NEUTRAL)
        ));

        //For sketchy possible 3 piece idea
        //CommandEventMap.put("ScoreConeLow", new InstantCommand(()->Pivot.getInstance().startPID(15),Pivot.getInstance()));

        CommandEventMap.put("ScoreCubeLow", new SequentialCommandGroup(
                                                new InstantCommand(()-> Intake.getInstance().outtake(.5), Intake.getInstance()),
                                                new InstantCommand(()->Intake.getInstance().startPID(45), Intake.getInstance())
                                                ));
        
        CommandEventMap.put("ShootCubeLow", new SequentialCommandGroup(
                                                new InstantCommand(()-> Intake.getInstance().outtake(), Intake.getInstance()),
                                                new InstantCommand(()->Intake.getInstance().startPID(90), Intake.getInstance())
                                                ));
        
        CommandEventMap.put("IntakeCube", CmdIntake());

        CommandEventMap.put("RetractIntake", new InstantCommand(()->Intake.getInstance().startPID(IntakeState.RETRACTED), Intake.getInstance()));

        // CommandEventMap.put("Climb", Commands.sequence(
        //     new InstantCommand(()-> Swerve.getInstance().stop(), Swerve.getInstance()),
        //     new CmdAutoBalance()
        // )
        // );

        //For sketchy possible 3 piece idea
        //CommandEventMap.put("RetractPivot", new InstantCommand(()->Pivot.getInstance().startPID(ArmPosition.NEUTRAL),Pivot.getInstance()));

        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            if (trajectoryName.contains("mid")) {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(AutoConstants.slowSpeed, AutoConstants.slowAcceleration)));
            } 
            else {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
            }
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

    public static CommandBase generateAuto(PathPlannerTrajectory trajectory) {
        return builder.fullAuto(trajectory);
    }

    public static CommandBase get(String name, boolean balance) {
        if (balance) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> Leds.getInstance().setPivotLeds(Colors.AUTO)),
                builder.fullAuto(trajectories.get(name)),
                new CmdAutoBalance(true)
            );
        }
        return new SequentialCommandGroup(
            new InstantCommand(() -> Leds.getInstance().setPivotLeds(Colors.AUTO)),
            builder.fullAuto(trajectories.get(name))
        );
    }

    public static boolean contains(String name) {
        return trajectories.containsKey(name);
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
            new PathConstraints(maxSpeed, 4),
            new PathPoint(start.getTranslation(), start.getRotation()), 
            new PathPoint(end.getTranslation(), end.getRotation())
            );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }
    
}