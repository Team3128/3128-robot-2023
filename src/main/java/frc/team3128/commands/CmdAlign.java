package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.TX_THRESHOLD;

import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdAlign extends PIDCommand {

    public CmdAlign(String camera) {
        super(
            new PIDController(alignKP,alignKI,alignKD),
            ()-> Vision.getInstance().getTX(camera),
            0,
            output -> Swerve.getInstance().drive(new Translation2d(), -output, false),
            Swerve.getInstance()
        );
        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(TX_THRESHOLD);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}