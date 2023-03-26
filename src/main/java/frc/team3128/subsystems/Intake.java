package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    private DutyCycleEncoder m_encoder;

    public static boolean objectPresent;

    private static Intake instance;

    public enum IntakeState {
        DEPLOYED(0),
        RETRACTED(100);

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
        m_intakeRollers.setNeutralMode(NeutralMode.Brake);

        m_intakePivot.setInverted(true);
        m_intakeRollers.setInverted(false);

        m_intakeRollers.enableVoltageCompensation(true);
        
    }

    public void configEncoders() {
        m_encoder = new DutyCycleEncoder(ENCODER_DIO_ID);
    }

    public double getAngle() {
        return MathUtil.inputModulus(-m_encoder.get() * ENCODER_CONVERSION_FACTOR_TO_DEGREES - ANGLE_OFFSET,-180, 180);
    }

    public void startPID(IntakeState desiredState) {
        startPID(desiredState.angle);
    }

    public void startPID(double setpoint) {
        setpoint = RobotContainer.DEBUG.getAsBoolean() ? this.setpoint.getAsDouble() : setpoint;
        setpoint = MathUtil.clamp(setpoint,0,105);
        setSetpoint(setpoint);
        enable();
    }

    @Override
    public void periodic() {
        super.periodic();
        NAR_Shuffleboard.addData("intake", "output current", m_intakeRollers.getStatorCurrent(), 4, 1);
        NAR_Shuffleboard.addData("intake", "input current", m_intakeRollers.getSupplyCurrent(), 4, 2);
        NAR_Shuffleboard.addData("intake", "output voltage", m_intakeRollers.getMotorOutputVoltage(), 5, 1);
        if (Math.abs(getCurrent()) > CURRENT_THRESHOLD + 20)
            set(STALL_POWER);
    }

    public double getCurrent() {
        return m_intakeRollers.getStatorCurrent();
    }

    public double getVoltage() {
        return m_intakeRollers.getMotorOutputVoltage();
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
        stopRollers();
    }

    public void moveIntake(double power) {
        disable();
        m_intakePivot.set(power);
    }

    // Roller Control
    public void intake() {
        set(ROLLER_POWER);
    }

    public boolean hasObjectPresent(){
        return getCurrent() > CURRENT_THRESHOLD;
    }

    public void outtake() {
        set(-OUTTAKE_POWER);
    }

    public void shoot() {
        set(-1);
    }

    public void set(double power) {
        m_intakeRollers.set(power);
    }

    public void stopRollers() {
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
