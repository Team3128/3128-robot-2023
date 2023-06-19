package frc.team3128.common.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.subsystems.Swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.common.swerve.CTREConfigs.*;
import static frc.team3128.common.swerve.FalconConversions.*;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final NAR_TalonFX angleMotor;
    private final NAR_TalonFX driveMotor;
    private final CANCoder angleEncoder;
    
    private Rotation2d lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "drivetrain");
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new NAR_TalonFX(moduleConstants.angleMotorID, "drivetrain");
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new NAR_TalonFX(moduleConstants.driveMotorID, "drivetrain");
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
        // Rotation2d oldAngle = getAngle();
        // angle = optimizeTurn(oldAngle, angle);  
        angleMotor.set(ControlMode.Position, degreesToFalcon(angle.getDegrees(), angleGearRatio));
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState) {
        double velocity = MPSToFalcon(desiredState.speedMetersPerSecond, wheelCircumference, driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond) / 12);
    }

    public void xLock(Rotation2d angle) {
        double desiredAngle = CTREModuleState.optimize(new SwerveModuleState(0, angle), getState().angle).angle.getDegrees();
        driveMotor.set(ControlMode.Velocity, 0);
        angleMotor.set(ControlMode.Position, degreesToFalcon(desiredAngle, angleGearRatio)); 
    }

    public void resetToAbsolute(){
        double absolutePosition = degreesToFalcon(makePositiveDegrees(getCanCoder().getDegrees()), angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
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
        return Rotation2d.fromDegrees(falconToDegrees(angleMotor.getSelectedSensorPosition(), angleGearRatio));
    }

    public SwerveModuleState getState(){
        double velocity = falconToMPS(driveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = falconToMeters(driveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
        if (driveMotor.getLastError().value == -3 && moduleNumber == 1) {
            Swerve.error = true;
        }
        // if (moduleNumber == 1) 
        //     System.out.println(position);
        // System.out.println(driveMotor.getLastError().value);
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(position, angle);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCancoderConfig());
    }

    private void configAngleMotor(){
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(swerveAngleFXConfig());
        angleMotor.setInverted(angleMotorInvert);
        angleMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(swerveDriveFXConfig());
        driveMotor.setInverted(driveMotorInvert);
        driveMotor.setNeutralMode(NeutralMode.Coast); 
        // driveMotor.setControlFramePeriod(ControlFrame.Control, angleContinuousCurrentLimit)
        driveMotor.setSelectedSensorPosition(0);
    }
    
}