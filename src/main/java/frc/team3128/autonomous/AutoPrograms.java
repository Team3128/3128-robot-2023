package frc.team3128.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public Vision vision;

    public AutoPrograms() {
        swerve = Swerve.getInstance();
        vision = Vision.getInstance();

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
            Trajectories.startScoringPoint(0, 0, false, ArmPosition.MID_CONE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(Vision.getInstance().getCamera(VisionConstants.FRONT).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false)
        ));

        auto.put("bottom_1Cone+1Cube", Commands.sequence(
            Trajectories.startScoringPoint(0, 0, false, ArmPosition.MID_CONE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(Vision.getInstance().getCamera(VisionConstants.FRONT).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.scoringPoint(0, 1, false, ArmPosition.MID_CUBE)
        ));

        auto.put("top_1Cone+1Cube", Commands.sequence(
            Trajectories.startScoringPoint(2, 2, false, ArmPosition.MID_CONE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(Vision.getInstance().getCamera(VisionConstants.FRONT).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.loadingPoint(AutoConstants.PICKUP_4, false),
            Trajectories.scoringPoint(2, 1, false, ArmPosition.MID_CUBE)
        ));

        auto.put("bottom_1Cone+Climb", Commands.sequence(
            Trajectories.startScoringPoint(0, 0, false, ArmPosition.MID_CONE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(Vision.getInstance().getCamera(VisionConstants.FRONT).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.climbPoint(false)
        ));

        auto.put("bottom_1Cone+1Cube+Climb", Commands.sequence(
            Trajectories.startScoringPoint(0, 0, false, ArmPosition.MID_CONE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> Vision.getInstance().getCamera(VisionConstants.FRONT).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(Vision.getInstance().getCamera(VisionConstants.FRONT).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.loadingPoint(AutoConstants.PICKUP_1, false),
            Trajectories.scoringPoint(0, 1, false, ArmPosition.MID_CUBE),
            Trajectories.climbPoint(true)
        ));
        /**
            * Middle Position Autos
        */

        auto.put("mid_1Cube+Climb", Commands.sequence(
            Trajectories.startScoringPoint(1, 1, true, ArmPosition.MID_CUBE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 180 : 0)),
            new RunCommand(()-> swerve.drive(new Translation2d(DriverStation.getAlliance() == Alliance.Red ? -0.35 : 0.35,0), 
                                    0, true), swerve).until(() -> vision.getCamera(VisionConstants.BACK).hasValidTarget()),
            new InstantCommand(()-> swerve.stop(), swerve),
            new InstantCommand(()-> swerve.resetOdometry(new Pose2d(vision.getCamera(VisionConstants.BACK).getPos().getTranslation(), swerve.getGyroRotation2d()))),
            Trajectories.climbPoint(false),
            new InstantCommand(() -> {if (vision.getCamera(VisionConstants.BACK).hasValidTarget()) 
                                        swerve.resetOdometry(new Pose2d(vision.getCamera(VisionConstants.BACK).getPos().getTranslation(), swerve.getGyroRotation2d()));})
                                        // ^^ use vision gyro
        ));

        auto.put("mid_1Cube", Commands.sequence(
            Trajectories.startScoringPoint(1, 1, false, ArmPosition.MID_CUBE),
            new InstantCommand(()-> swerve.zeroGyro(DriverStation.getAlliance() == Alliance.Red ? 0 : 180))
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
        //String selectedAutoName = "mid_1Cube"; //uncomment and change this for testing without opening Narwhal Dashboard

        // if (selectedAutoName == null) {
        //     return null;
        // }

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //      selectedAutoName = "b_" + selectedAutoName;
        // }
        //  else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //      selectedAutoName = "r_" + selectedAutoName;
        // }
        //if (vision.getCamera(VisionConstants.BACK).hasValidTarget()) resetPose = vision.getCamera(VisionConstants.BACK).getPos();
        return auto.get(selectedAutoName);
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