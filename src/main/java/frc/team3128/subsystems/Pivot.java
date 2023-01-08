package frc.team3128.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

import static frc.team3128.Constants.PivotConstants.*;

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

        // This is bad because of hierachy problem in NAR_CANSparkMax
        // Not sure what frame periods are anyways

        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 0); 
        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0);
        m_rotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0);
        m_rotateMotor.setControlFramePeriodMs(0);
    }

    private void configEncoders() {
        // m_encoder =  m_rotateMotor.getEncoder();
    }

    public void stop() {
        m_rotateMotor.set(0);
    }

    public void zeroEncoder() {
        m_rotateMotor.setEncoderPosition(0);
    }

    @Override
    public void periodic() {

    }

    @Override
    protected void useOutput(double output, double setpoint) {
        
    }

    @Override
    protected double getMeasurement() {
        // Will get current angle of pivot
        return 0;
    }
    
}
