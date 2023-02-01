package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.BalanceConstants.*;

public class CmdGyroBalance extends PIDCommand {
    public int plateauCount;

    public CmdGyroBalance() {
        super(
            new PIDController(turnKP,turnKI,turnKD),
            () -> Swerve.getInstance().getPitch(),
            0,
            output -> Swerve.getInstance().drive(new Translation2d(output,0), 0,true),
            Swerve.getInstance()
        );

        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(TURN_TOLERANCE);
        plateauCount = 0;
    }

    @Override
    public void initialize() {
        super.initialize();
        //m_setpoint = ()-> MathUtil.inputModulus(Swerve.getInstance().getPitch(),-180,180);
    }

    public void execute() {
        super.execute();
        SmartDashboard.putBoolean("atsetpoint mason bad", m_controller.atSetpoint());
        if (Math.abs(Swerve.getInstance().getPitch()) <= TURN_TOLERANCE) {
            plateauCount += 1;
        } else {
            plateauCount = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return plateauCount >= 5;
    }
}