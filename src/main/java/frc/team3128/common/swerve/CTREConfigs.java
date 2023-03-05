package frc.team3128.common.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import static frc.team3128.Constants.SwerveConstants.*;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            driveEnableCurrentLimit, 
            driveContinuousCurrentLimit, 
            drivePeakCurrentLimit, 
            drivePeakCurrentDuration);

        config.slot0.kP = driveKP;
        config.slot0.kI = driveKI;
        config.slot0.kD = driveKD;
        config.slot0.kF = driveKF;        
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.closedloopRamp = closedLoopRamp;
        return config;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            angleEnableCurrentLimit, 
            angleContinuousCurrentLimit, 
            anglePeakCurrentLimit, 
            anglePeakCurrentDuration);

        angleConfig.slot0.kP = angleKP;
        angleConfig.slot0.kI = angleKI;
        angleConfig.slot0.kD = angleKD;
        angleConfig.slot0.kF = angleKF;
        angleConfig.supplyCurrLimit = angleSupplyLimit;
        angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static CANCoderConfiguration swerveCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // used to be unsigned 
        config.sensorDirection = canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }

    public static CANCoderConfiguration pivotCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // used to be unsigned 
        config.sensorDirection = canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        return config;
    }
}
