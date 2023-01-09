package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

import static frc.team3128.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Pivot extends PIDSubsystem {

    private static Pivot instance;
    private NAR_CANSparkMax m_rotateMotor;
    private SparkMaxRelativeEncoder m_encoder;
    public double m_ff;
    public double m_setpoint;

    public Pivot() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
    }

    public static synchronized Pivot getInstance(){
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    private void configMotors() {
        m_rotateMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 0); 
        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0);
        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
        m_rotateMotor.setControlFramePeriodMs(0);
    }

    private void configEncoders() {
        m_encoder =  (SparkMaxRelativeEncoder) m_rotateMotor.getEncoder();
        m_encoder.setPositionConversionFactor(ENC_CONV);
    }

    public void stop() {
        m_rotateMotor.set(0);
    }

    public void zeroEncoder() {
        m_rotateMotor.setEncoderPosition(0);
    }

    @Override
    public void periodic() {

        super.periodic();
    }

    public void startPID(double angle) {
        // if (RobotContainer.DEBUG) {
        //     angle = m_setpoint;
        // }

        super.setSetpoint(angle);
        getController().setTolerance(TOLERANCE);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = m_ff; //Need to calculate this
        double voltageOutput = output + ff;

        m_rotateMotor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
       return m_rotateMotor.getSelectedSensorPosition() + MIN_ANGLE;
    }
    
}
