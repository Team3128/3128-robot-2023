package frc.team3128.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.FieldConstants.*;

import java.util.ArrayList;
import static frc.team3128.Constants.AutoConstants.*;

import static frc.team3128.Constants.TrajectoryConstants.*;

import frc.team3128.autonomous.Trajectories;
import frc.team3128.subsystems.Swerve;

public class CmdTrajectory extends CommandBase {

    private final Swerve swerve;
    private final int index;
    private CommandBase trajCommand;

    public CmdTrajectory(int index) {
        swerve = Swerve.getInstance();
        this.index = index;
        addRequirements(swerve);
    }

    private PathPoint generatePoint(Pose2d pose, Rotation2d heading, double headingLength) {
        return generatePoint(pose.getTranslation(), pose.getRotation(), heading, headingLength);
    }

    private PathPoint generatePoint(Translation2d translation, Rotation2d holonomicAngle, Rotation2d heading, double headingLength) {
        final PathPoint point =  new PathPoint(allianceFlip(translation), allianceFlip(heading), allianceFlip(holonomicAngle));
        point.prevControlLength = headingLength;
        point.nextControlLength = headingLength;
        return point;
    }

    private boolean pastPoint(Translation2d start, double condition) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            return start.getX() < condition;
        }
        return start.getX() > FIELD_X_LENGTH - condition;
    }

    private ArrayList<PathPoint> generatePoses() {
        final ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
        final Translation2d start = swerve.getPose().getTranslation();
        final Rotation2d holonomicAngle = allianceFlip(END_POINTS[index].getRotation());
        final PathPoint startPoint = new PathPoint(start, allianceFlip(HEADING), swerve.getGyroRotation2d());
        final boolean topPath = start.getY() >= (POINT_2A.getY() + POINT_2B.getY()) / 2;
        final boolean skipLastPoint = (topPath && index == 8) || (!topPath && index == 0);
        startPoint.nextControlLength = 0.1;
        pathPoints.add(startPoint);
        if (!pastPoint(start, CONDITION_1)) pathPoints.add(generatePoint(POINT_1, holonomicAngle, HEADING, 1));
        if (!pastPoint(start, CONDITION_2)) pathPoints.add(generatePoint(topPath ? POINT_2B : POINT_2A, holonomicAngle, HEADING, 1));
        if (!pastPoint(start, CONDITION_3) && !skipLastPoint) pathPoints.add(generatePoint(topPath ? POINT_3B : POINT_3A, holonomicAngle, HEADING, 0.5));
        pathPoints.add(generatePoint(END_POINTS[index], HEADING, 0.1));
        
        return pathPoints;
    }

    @Override
    public void initialize() {
        final PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            pathConstraints, generatePoses()
        );
        trajCommand = Trajectories.generateAuto(trajectory);
        trajCommand.schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return !trajCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
