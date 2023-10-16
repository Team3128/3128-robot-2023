package frc.team3128.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.commands.CmdAutoBalance2;
import frc.team3128.commands.CmdMoveArm;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;
import static edu.wpi.first.wpilibj2.command.Commands.*;

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
        String[] autoStrings = new String[] {
                                            //Blue Autos
                                                //Cable
                                                "b_cable_1Cone+1Cube","b_cable_1Cone+2Cube", "b_cable_1Cone+2Cube+Climb",
                                                //Mid
                                                "b_mid_1Cone+Climb","b_mid_1Cone+1Cube+Climb",
                                                //Hp
                                                // "b_hp_1Cone+1Cube",
                                                "b_cable_1Cone+2Cube",
                                            
                                            //Red Autos
                                                //Cable
                                                "r_cable_1Cone+1Cube","r_cable_1Cone+2Cube",
                                                //Mid
                                                "r_mid_1Cone+1Cube+Climb",
                                                //Hp
                                                // "r_hp_1Cone+1Cube",
                                                "r_cable_1Cone+2Cube",
                                                "ScuffedClimb"

                                            };
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
       String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
       return sequence(
        runOnce(()-> Swerve.getInstance().zeroGyro(DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)),
        new CmdMoveArm(ArmPosition.TOP_CONE),
        runOnce(() -> Manipulator.getInstance().outtake()),
        waitSeconds(.5),
        runOnce(() -> Manipulator.getInstance().stopRoller()),
        new CmdMoveArm(ArmPosition.NEUTRAL),
        new CmdAutoBalance()
    );

    //    if (selectedAutoName == "ScuffedClimb") {
    //        return sequence(
    //             runOnce(()-> Swerve.getInstance().zeroGyro(DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)),
    //             new CmdMoveArm(ArmPosition.TOP_CONE),
    //             runOnce(() -> Manipulator.getInstance().outtake()),
    //             waitSeconds(.5),
    //             runOnce(() -> Manipulator.getInstance().stopRoller()),
    //             new CmdMoveArm(ArmPosition.NEUTRAL),
    //             new CmdAutoBalance()
    //        );
    //    }
    //     // String selectedAutoName = "b_cable_1Cone+1Cube"; //uncomment and change this for testing without opening Narwhal Dashboard
        // if (selectedAutoName == null || !Trajectories.contains(selectedAutoName)) {
        //     return sequence(
        //         new CmdMoveArm(ArmPosition.TOP_CONE),
        //         runOnce(() -> Manipulator.getInstance().outtake()),
        //         waitSeconds(.5),
        //         runOnce(() -> Manipulator.getInstance().stopRoller()),
        //         new CmdMoveArm(ArmPosition.NEUTRAL)
        //         );
        // }
        // // SmartDashboard.putString(selectedAutoName, selectedAutoName);

        // return Trajectories.get(selectedAutoName, selectedAutoName.contains("Climb"));
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