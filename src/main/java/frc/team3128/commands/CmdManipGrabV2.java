package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.team3128.Constants.ManipulatorConstants.STALL_POWER;
import frc.team3128.subsystems.V2Manipulator;


public class CmdManipGrabV2 extends SequentialCommandGroup {

    public CmdManipGrabV2(boolean cone, boolean shelf) {
        var manipulator = V2Manipulator.getInstance();
        addCommands(
            new InstantCommand(()-> manipulator.intake(cone, shelf), manipulator),
            new WaitCommand(0.1),
            new WaitUntilCommand(()-> manipulator.hasObjectPresent()),
            new WaitCommand(cone ? 0.5 : 0), // TODO test
            new InstantCommand(()-> manipulator.setRollerPower(STALL_POWER), manipulator),
            new InstantCommand(() -> V2Manipulator.objectPresent = true)
        );
    }
}
