package frc.team3128.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.subsystems.Swerve;
import frc.team3128.Constants.BalanceConstants;
import frc.team3128.Constants.BalanceConstants.*;

public class CmdDriveUp extends SequentialCommandGroup{
    
    public CmdDriveUp() {

        addCommands(
            new CmdMove()
        );
    }
}
