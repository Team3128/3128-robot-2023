package frc.team3128.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdScoreOptimized;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam, Leo Lesmes
 */

public class AutoPrograms {
    private HashMap<String, Command> auto;
    public static Swerve swerve;

    public AutoPrograms() {
        swerve = Swerve.getInstance();

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        // String[] autoStrings = new String[] {"top_1Cone", "top_1Cone+1Cube", "top_1Cone+1Cube+Climb",
        //                                     "mid_1Cone", "mid_1Cone+Climb",
        //                                     "bottom_1Cone", "bottom_1Cone+1Cube", "bottom_1Cone+1Cube+Climb", "TestAuto"
        //                                     }; // naming scheme kinda mid, but its grown on me and now I love it so much
        
        auto = new HashMap<String, Command>();
        
        /**
            * Bottom Position Autos
        */

        auto.put("bottom_1Cone", Commands.sequence(
            Trajectories.scoringPoint(0, 0, false, ArmPosition.TOP_CONE),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false)
        ));

        auto.put("bottom_1Cone+1Cube", Commands.sequence(
            Trajectories.scoringPoint(0, 0, false, ArmPosition.TOP_CONE),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.scoringPoint(0, 1, false, ArmPosition.TOP_CUBE)
        ));

        auto.put("top_1Cone+1Cube", Commands.sequence(
            Trajectories.scoringPoint(2, 0, false, ArmPosition.TOP_CONE),
            Trajectories.loadingPoint(AutoConstants.PICKUP_4, false),
            Trajectories.scoringPoint(2, 1, false, ArmPosition.TOP_CUBE)
        ));

        auto.put("bottom_1Cone+Climb", Commands.sequence(
            Trajectories.scoringPoint(0, 0, false, ArmPosition.TOP_CONE),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.climbPoint(false)
        ));

        auto.put("bottom_1Cone+1Cube+Climb", Commands.sequence(
            Trajectories.scoringPoint(0, 0, false, ArmPosition.TOP_CONE),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.scoringPoint(0, 1, false, ArmPosition.TOP_CUBE),
            Trajectories.climbPoint(true)
        ));
        /**
            * Middle Position Autos
        */

        auto.put("mid_1Cone+Climb", Commands.sequence(
            Trajectories.scoringPoint(1, 0, false, ArmPosition.TOP_CONE),
            Trajectories.climbPoint(false)
        ));

        
        var array = auto.keySet();

        var arrayCopy = new String[array.size()];
        int index = 0;
        for (String x : array) {
            arrayCopy[index] = x;
            index++;
        }

        NarwhalDashboard.addAutos(arrayCopy);
    }

    public Command getAutonomousCommand() {
       String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "bottom_1Cone+1Cube"; //uncomment and change this for testing without opening Narwhal Dashboard
        Vision vision = Vision.getInstance();

        // if (selectedAutoName == null) {
        //     return null;
        // }

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //      selectedAutoName = "b_" + selectedAutoName;
        // }
        //  else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //      selectedAutoName = "r_" + selectedAutoName;
        // }
        Pose2d resetPose = vision.getCamera(VisionConstants.FRONT).getPos();
        Swerve.getInstance().zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180);
        //if (vision.getCamera(VisionConstants.BACK).hasValidTarget()) resetPose = vision.getCamera(VisionConstants.BACK).getPos();
        Swerve.getInstance().resetOdometry(new Pose2d(resetPose.getTranslation(), Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)));
        
        return Commands.sequence(
            new WaitUntilCommand(()-> vision.getCamera(VisionConstants.FRONT).hasValidTarget()),
            auto.get(selectedAutoName)
        );
        // return Trajectories.get(selectedAutoName);
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