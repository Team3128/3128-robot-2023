package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.swerve.FalconConversions;

import static frc.team3128.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Intake subsystem controls rolling wheels of bars, rotation of bars, and conveyer belt in "Star of David" Box.
 */

public class Intake extends PIDSubsystem {

    public enum IntakeRotationState {
        FLOOR(0), RAISED(90);
    
        private double degrees;
        IntakeRotationState(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }

    }

    //Motors
    private NAR_TalonSRX m_intakeWheels;
    private NAR_TalonFX m_intakeRotator;
    private NAR_TalonSRX m_serializer;

    //TODO: Make the feedforward based on the weight and angle of the intake arm
    private double m_ff = 0.0;

    private static Intake instance;

    private Intake() {
        super(new PIDController(kP, kI, kD));
        m_intakeRotator.setNeutralMode(NeutralMode.Brake);
    }

    public static Intake getInstance() {
        if(instance == null){
            instance = new Intake() ;  
        }
        return instance;
    }

    public void configMotors() {
        m_intakeWheels = new NAR_TalonSRX(INTAKE_WHEELS_ID);
        m_intakeRotator = new NAR_TalonFX(INTAKE_ROTATOR_ID);
        m_serializer = new NAR_TalonSRX(SERIALIZER_ID);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double voltageOutput = m_ff*setpoint+output;

        //TODO: Consider ControlMode.Position
        m_intakeRotator.set(MathUtil.clamp(voltageOutput, -1, 1));
        
    }

    @Override
    protected double getMeasurement() {
        return m_intakeRotator.getSelectedSensorPosition();
    }

    public void setRotationSetpoint(IntakeRotationState state) {
        enable();
        setSetpoint(FalconConversions.degreesToFalcon(state.getDegrees(), ROTATOR_GEAR_RATIO));
        getController().setTolerance(ROTATOR_TOLERANCE);
    }

    public void stopRotation() {
        disable();
        m_intakeRotator.set(0);
    }

    //Power On/Off Method for Wheel Motors and Conveyer Belt Motors

    public void enableWheels() {
        m_intakeWheels.set(WHEELS_POWER);
    }

    public void disableWheels() {
        m_intakeWheels.set(0);
    }

    public void enableSerializer() {
        m_serializer.set(SERIALIZER_POWER);
    }

    public void disableSerializer() {
        m_serializer.set(0);
    }
}
