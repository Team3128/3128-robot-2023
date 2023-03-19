package frc.team3128.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam, Leo Lesmes
 */

public class AutoPrograms {
    private HashMap<String, Command> auto;
    public Swerve swerve;
    public Vision vision;

    public AutoPrograms() {
        swerve = Swerve.getInstance();
        vision = Vision.getInstance();

        // Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {

        auto = new HashMap<String, Command>();
        
        /**
            * Bottom Position Autos
        */
        auto.put("DEFAULT", Commands.sequence(
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(true)
        ));

        auto.put("bottom_1pc+mobility", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(true),
            Trajectories.movePoint(AutoConstants.PICKUP_1)
        ));

        auto.put("top_1pc+mobility", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(true),
            Trajectories.movePoint(AutoConstants.PICKUP_4)
        ));

        auto.put("bottom_2pc", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(false),
            Trajectories.intakePoint(AutoConstants.PICKUP_1),
            Trajectories.scoreIntake(0, 1)
        ));

        auto.put("top_2pc", Commands.sequence(
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(true),
            Trajectories.intakePoint(AutoConstants.PICKUP_4),
            Trajectories.scoreIntake(2, 1)
        ));

        auto.put("bottom_2pc+Climb", Commands.sequence(
            // Trajectories.startScoringPoint(true, ArmPosition.TOP_CONE),
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.resetOdometry(false),
            Trajectories.intakePoint(AutoConstants.PICKUP_1),
            Trajectories.climbPoint(false, true, true)
        ));

        auto.put("top_2pc+Climb", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(true),
            Trajectories.intakePoint(AutoConstants.PICKUP_4),
            Trajectories.climbPoint(false, false, true)
        ));

        auto.put("bottom_3pc", Commands.sequence(
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(false),
            Trajectories.intakePoint(AutoConstants.PICKUP_1),
            Trajectories.scoreIntake(0, 0),
            Trajectories.intakePointRamp(AutoConstants.PICKUP_2),
            Trajectories.scoreIntake(0, 1)
        ));

        auto.put("top_3pc", Commands.sequence(
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(false),
            Trajectories.intakePoint(AutoConstants.PICKUP_4),
            Trajectories.scoreIntake(2, 1),
            Trajectories.intakePointRamp(AutoConstants.PICKUP_3),
            Trajectories.scoreIntake(2, 2)
        ));

        /**
            * Middle Position Autos
        */

        auto.put("mid_1Cube+Climb", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(true, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(false),
            Trajectories.climbPoint(true, false, false)
        ));
        
        auto.put("mid_2pc+Climb", Commands.sequence(
            new InstantCommand(()-> Trajectories.autoSpeed = 2.5),
            Trajectories.startScoringPoint(false, ArmPosition.TOP_CONE),
            Trajectories.resetOdometry(false),
            Trajectories.intakePointSpecial(AutoConstants.PICKUP_2),
            Trajectories.climbPoint(false, true, true)
            //Outtake

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
        // String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        String selectedAutoName = "bottom_2pc+Climb"; //uncomment and change this for testing without opening Narwhal Dashboard

        if (selectedAutoName == null) {
            return auto.get("DEFAULT");
        }

        return auto.get(selectedAutoName);
    }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}