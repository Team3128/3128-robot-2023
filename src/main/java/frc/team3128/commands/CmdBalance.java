package frc.team3128.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.team3128.Constants.FieldConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.subsystems.Swerve;

public class CmdBalance extends PIDCommand {

    public static boolean DIRECTION;

    public CmdBalance () {
        super(
            new PIDController(1,0,0),
            () -> Swerve.getInstance().getPose().getX(),
            //() -> DriverStation.getAlliance() == Alliance.Red ? 12.55 : 3.95,
            () -> DriverStation.getAlliance() == Alliance.Red ? FIELD_X_LENGTH - (chargingStationInnerX + chargingStationOuterX) / 2.0 : (chargingStationInnerX + chargingStationOuterX) / 2.0,
            output -> Swerve.getInstance().drive(new Translation2d(output - AutoConstants.BALANCE_FF,0), 0,true),
            Swerve.getInstance()
        );
        m_controller.setTolerance(SwerveConstants.DRIVE_TOLERANCE);

    }

    @Override
    public void initialize() {
        super.initialize();
        DIRECTION = m_controller.calculate(Swerve.getInstance().getPose().getX()) > 0;
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

}