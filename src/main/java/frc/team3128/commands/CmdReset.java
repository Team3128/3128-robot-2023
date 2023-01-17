package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

public class CmdReset extends CommandBase {

    Swerve swerve = Swerve.getInstance();
    NAR_Camera cam = Vision.getInstance().getCamera(VisionConstants.SHOOTER.hostname);

    public CmdReset() {}

    @Override
    public void initialize() {
        if (cam.hasValidTarget()) {
            swerve.resetOdometry(cam.getPos());
        }
    }
}
