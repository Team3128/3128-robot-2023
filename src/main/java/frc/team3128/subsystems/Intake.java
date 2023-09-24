package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_TalonSRX;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.subsystems.NAR_PIDSubsystem;
import common.utility.Controller.PController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.team3128.RobotContainer;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.TelescopeConstants;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Intake extends NAR_PIDSubsystem {

    // Motors
    private NAR_CANSparkMax m_intakePivot;
    public NAR_TalonSRX m_intakeRollers;

    // Encoder
    private DutyCycleEncoder m_encoder;

    public static boolean objectPresent;

    private static Intake instance;

    public enum IntakeState {
        DEPLOYED(-3),
        RETRACTED(100);

        public double angle;

        private IntakeState(final double angle) {
            this.angle = angle;
        }
    }

    private Intake() {
        super(new PController(kS, kG, kP, kI, kD));


        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));

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
        m_intakePivot = new NAR_CANSparkMax(INTAKE_PIVOT_ID);
        m_intakeRollers = new NAR_TalonSRX(INTAKE_ROLLERS_ID);

        m_intakePivot.setNeutralMode(Neutral.BRAKE);
        m_intakeRollers.setNeutralMode(Neutral.BRAKE);

        m_intakePivot.setInverted(true);
        m_intakeRollers.setInverted(false);
    }

    public void configEncoders() {
        m_encoder = new DutyCycleEncoder(ENCODER_DIO_ID);
    }

    public double getAngle() {
        return MathUtil.inputModulus(-m_encoder.get() * ENCODER_CONVERSION_FACTOR_TO_DEGREES - ANGLE_OFFSET,-180, 180);
    }

    public double getCurrent() {
        return m_intakeRollers.getStatorCurrent();
    }

    @Override
    protected void useOutput(double output) {
        m_intakePivot.set(MathUtil.clamp(output / 12.0, -1, 1));
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


    public boolean hasObjectPresent(){
        return getCurrent() > ABSOLUTE_THRESHOLD;
    }

    // Roller Control
    public void intake() {
        set(ROLLER_POWER);
    }

    public void outtake() {
        set(-OUTTAKE_POWER);
    }

    public void stallPower() {
        set(STALL_POWER);
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

    public void startPID(IntakeState desiredState) {
        startPID(desiredState.angle);
    }
    
    public void initShuffleboard() {
        NAR_Shuffleboard.addData("intake", "output current", m_intakeRollers.getStatorCurrent(), 4, 1);
    }
}
