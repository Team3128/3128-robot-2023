package frc.team3128.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Swerve;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public static Swerve swerve;

    public AutoPrograms() {
        swerve = Swerve.getInstance();

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {"top_1Cone", "top_1Cone+1Cube", "top_1Cone+1Cube+Climb",
                                            "mid_1Cone", "mid_1Cone+Climb",
                                            "bottom_1Cone", "bottom_1Cone+1Cube", "bottom_1Cone+1Cube+Climb",
                                            };
        
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
       String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        //String selectedAutoName = "1Cone_bottom"; //uncomment and change this for testing without opening Narwhal Dashboard

        if (selectedAutoName == null) {
            return null;
        }

        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            selectedAutoName = "b_" + selectedAutoName;
        }
        else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            selectedAutoName = "r_" + selectedAutoName;
        }

        return Trajectories.get(selectedAutoName);
    }
    
    // /** 
    //  * Follow trajectory and intake balls along the path
    //  */
    // private SequentialCommandGroup IntakePathCmd(String trajectory) {
    //     ParallelDeadlineGroup movement = new ParallelDeadlineGroup(
    //                                         trajectoryCmd(trajectory), 
    //                                         new ScheduleCommand(new CmdExtendIntakeAndRun()));
    //     return new SequentialCommandGroup(new InstantCommand(intake::ejectIntake, intake), movement);
    // }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}