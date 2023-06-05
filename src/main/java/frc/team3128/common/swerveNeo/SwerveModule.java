package frc.team3128.common.swerveNeo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax.EncoderType;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.common.swerveNeo.SwerveConversions.*;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final NAR_CANSparkMax angleMotor;
    private final NAR_CANSparkMax driveMotor;
    private final CANCoder angleEncoder;
    
    private Rotation2d lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new NAR_CANSparkMax(moduleConstants.angleMotorID, EncoderType.Relative, MotorType.kBrushless, SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new NAR_CANSparkMax(moduleConstants.driveMotorID, EncoderType.Relative, MotorType.kBrushless, SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.025)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleMotor.set(degreesToRotations(angle.getDegrees(), angleGearRatio), ControlType.kPosition);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState) {
        double velocity = MPSToRPM(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
        driveMotor.set(velocity, ControlType.kVelocity, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    public void xLock(Rotation2d angle) {
        double desiredAngle = CTREModuleState.optimize(new SwerveModuleState(0, angle), getState().angle).angle.getDegrees();
        driveMotor.set(0, ControlType.kVelocity);
        angleMotor.set(degreesToRotations(desiredAngle, angleGearRatio), ControlType.kPosition); 
    }

    public void resetToAbsolute(){
        double absolutePosition = degreesToRotations(makePositiveDegrees(getCanCoder().getDegrees()), angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public double makePositiveDegrees(double angle) {
        return MathUtil.inputModulus(angle, 0, 360);
    }

    public double makePositiveDegrees(Rotation2d angle){
        return makePositiveDegrees(angle.getDegrees());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition() - angleOffset);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(rotationsToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    public SwerveModuleState getState(){
        double velocity = RPMToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(position, angle);
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.swerveCancoderConfig());
    }

    private void configAngleMotor(){
        angleMotor.setSmartCurrentLimit(currentLimit);
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setIdleMode(IdleMode.kCoast);
        //angleMotor.enableContinuousInput(-180, 180, degreesToRotations(1, angleGearRatio));
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.setSmartCurrentLimit(currentLimit);
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setIdleMode(IdleMode.kCoast); 
        driveMotor.setSelectedSensorPosition(0);
    }
    
}