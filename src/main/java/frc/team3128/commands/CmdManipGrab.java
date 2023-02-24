package frc.team3128.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team3128.subsystems.Manipulator;

public class CmdManipGrab extends SequentialCommandGroup {

    public CmdManipGrab(boolean cone) {
        var manipulator = Manipulator.getInstance();
        addCommands(
            new InstantCommand(()-> {
                if (cone) 
                    manipulator.intakeCones();
                else
                    manipulator.intakeCubes();
            }, manipulator),
            new WaitCommand(0.02),
            new WaitUntilCommand(()-> manipulator.hasObjectPresent()),
            new WaitCommand(cone ? 0.1 : 0),
            new InstantCommand(()-> manipulator.setRollerPower(0.2), manipulator)
        );
    }
}
