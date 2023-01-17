package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.TX_THRESHOLD;

public class CmdInPlaceTurn extends PIDCommand {

    private double degrees;

    private boolean llInterrupt = false;

    private String camera;

    public CmdInPlaceTurn(double degrees, String camera) {
        this(degrees);
        llInterrupt = true;
        this.camera = camera;
    }

    public CmdInPlaceTurn(double degrees) {
        super(
            new PIDController(turnKP,turnKF,turnKD),
            () -> Swerve.getInstance().getHeading(),
            0,
            output -> Swerve.getInstance().drive(new Translation2d(), -output, false),
            Swerve.getInstance()
        );

        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(TX_THRESHOLD);

        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_setpoint = ()-> MathUtil.inputModulus(Swerve.getInstance().getHeading() + degrees,-180,180);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint() || (llInterrupt && Vision.getInstance().hasValidTarget(camera));
    }
}