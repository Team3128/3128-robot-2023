package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.SwerveConstants.*;

public class CmdGyroBalance extends PIDCommand {

    public CmdGyroBalance() {
        super(
            new PIDController(turnKP,turnKF,turnKD),
            () -> Swerve.getInstance().getPitch(),
            0,
            output -> Swerve.getInstance().drive(new Translation2d(output,0), 0,true),
            Swerve.getInstance()
        );

        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(TURN_TOLERANCE);
    }

    @Override
    public void initialize() {
        super.initialize();
        //m_setpoint = ()-> MathUtil.inputModulus(Swerve.getInstance().getPitch(),-180,180);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
