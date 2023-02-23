package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Intake extends PIDSubsystem {

    // Motors
    private NAR_CANSparkMax m_intakePivot;
    private NAR_TalonSRX m_intakeRollers;

    private DoubleSupplier kF;
    private DoubleSupplier setpoint, power;

    // Encoder
    private SparkMaxRelativeEncoder m_encoder;

    public boolean objectPresent;

    private static Intake instance;

    public enum IntakeState {
        DEPLOYED(0),
        RETRACTED(90),
        SEMI_DEPLOYED(60),
        STOWED(185);

        public double angle;

        private IntakeState(final double angle) {
            this.angle = angle;
        }
    }

    private Intake() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        configEncoders();
        getController().setTolerance(INTAKE_TOLERANCE);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    // Config
    public void configMotors() {
        m_intakePivot = new NAR_CANSparkMax(INTAKE_PIVOT_ID, MotorType.kBrushless);
        m_intakeRollers = new NAR_TalonSRX(INTAKE_ROLLERS_ID);

        m_intakePivot.setIdleMode(IdleMode.kBrake);

        m_intakePivot.setInverted(true);
        m_intakeRollers.setInverted(false);
        
    }

    public void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_intakePivot.getEncoder();
        m_encoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR_TICKS_TO_DEGREES);
        m_encoder.setInverted(false);
    }

    public void resetEncoders(double position) {
        m_intakePivot.setEncoderPosition(position);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void startPID(IntakeState desiredState) {
        startPID(desiredState.angle);
    }

    public void startPID(double setpoint) {
        setpoint = RobotContainer.DEBUG.getAsBoolean() ? this.setpoint.getAsDouble() : setpoint;
        setpoint = setpoint > 190 ? 190 : setpoint;
        setpoint = setpoint < 0 ? 0 : setpoint;
        setSetpoint(setpoint);
        enable();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        NAR_Shuffleboard.addData("intake", "output current", m_intakeRollers.getStatorCurrent(), 4, 1);
        NAR_Shuffleboard.addData("intake", "input current", m_intakeRollers.getSupplyCurrent(), 4, 2);
        NAR_Shuffleboard.addData("intake", "output voltage", m_intakeRollers.getMotorOutputVoltage(), 5, 1);
    }

    public double getCurrent() {
        return m_intakeRollers.getStatorCurrent();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        var ff = Math.cos(Units.degreesToRadians(setpoint)) * kF.getAsDouble();
        // Setpoint in radians, and velocity in radians per second
        double outputVoltage = output + ff;

        m_intakePivot.set(MathUtil.clamp(outputVoltage / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return getAngle();
    }

    public void stop() {
        disable();
        m_intakePivot.set(0);
        disableRollers();
    }

    public void setIntake(double power) {
        disable();
        m_intakePivot.set(power);
    }

    // Roller Control
    public void enableRollersForward() {
        enableRollers(ROLLER_POWER);
    }

    public boolean hasObjectPresent(){
        boolean objectPresent = getCurrent() > CURRENT_THRESHOLD;
        enableRollers(objectPresent ? 0.3 : 0.5);
        return objectPresent;
    }

    public void enableRollersReverse() {
        enableRollers(-ROLLER_POWER);
    }

    public void enableRollers(double wheelsPower) {
        m_intakeRollers.set(wheelsPower);
    }

    public void disableRollers() {
        m_intakeRollers.set(0);
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("intake", "Intake Angle", () -> getAngle(), 0, 0);
        NAR_Shuffleboard.addData("intake", "Pivot Velocity", () -> m_intakePivot.getSelectedSensorVelocity(), 2, 0);
        NAR_Shuffleboard.addData("intake", "Angle Setpoint", () -> getSetpoint(), 3, 0);
        NAR_Shuffleboard.addData("intake", "Roller Velocity", () -> m_intakeRollers.getSelectedSensorVelocity() / 4096, 4, 0);

        setpoint = NAR_Shuffleboard.debug("intake", "setpoint", 0, 1,2);
        NAR_Shuffleboard.addData("intake", "IsEnabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData("intake", "atSetpoint", ()-> getController().atSetpoint(), 1, 1);
        power = NAR_Shuffleboard.debug("intake", "power", 0.1, 1, 4);
        kF = NAR_Shuffleboard.debug("intake", "kF", IntakeConstants.kF, 2, 2);

        NAR_Shuffleboard.addComplex("intake", "PID Controller", m_controller, 0, 2);
    }
}
