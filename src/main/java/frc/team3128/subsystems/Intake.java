package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Intake extends PIDSubsystem {

    // Motors
    private NAR_CANSparkMax m_intakePivot;
    private NAR_TalonSRX m_intakeRollers;

    // Sensors
    private DigitalInput m_intakeSensor;
    //private DigitalInput m_coneSensor;
    //private DigitalInput m_intakeSensorLeft;
    //private DigitalInput m_intakeSensorRight;

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

        private double angle;

        private IntakeState(final double angle) {
            this.angle = angle;
        }

        public double getAngleSetpoint() {
            return angle;
        }
    }

    private Intake() {
        super(new PIDController(kP, kI, kD));

        configMotors();
        // configSensors();
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

    public void configSensors() {
        m_intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
        //m_coneSensor = new DigitalInput(CONE_SENSOR_ID);
        //m_intakeSensorLeft = new DigitalInput(INTAKE_SENSOR_LEFT_ID);
        //m_intakeSensorRight = new DigitalInput(INTAKE_SENSOR_RIGHT_ID);
    }

    public void configEncoders() {
        m_encoder = (SparkMaxRelativeEncoder) m_intakePivot.getEncoder();
        m_encoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR_TICKS_TO_DEGREES);
    }

    public void resetEncoders(double position) {
        m_intakePivot.setEncoderPosition(position);
        // m_intakePivot.setEncoderPosition(90);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void startPID(IntakeState desiredState) {
        startPID(desiredState.getAngleSetpoint());
    }

    public void startPID(double setpoint) {
        setSetpoint(setpoint);
        // setSetpoint(this.setpoint.getAsDouble());
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

    public boolean checkObjectPresent() {
        if (getCurrent() >= CURRENT_THRESHOLD) {
            enableRollers(0.3);
            objectPresent = true;
        } else {
            enableRollers(0.5);
            objectPresent = false;
        }

        return objectPresent;
    }

    public boolean intakeCube() {
        enableRollers(-0.5);
        return hasObject();
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

    // Sensor Methods
    /*
     * Idea is that if something is in the intake, then the left sensor will always
     * be true as something will be covering it,
     * but depending on if there is a cube or cone in the intake, the right sensor
     * will either be true or false.
     * 
     * Pretty sure that the .get() function returns whether the beam is broken or
     * not
     */

    public boolean hasObject() {
        return (!m_intakeSensor.get());
    }

    // TODO: give this a better name
    // public boolean hasConeOnPole() {
    //     return !m_coneSensor.get();
    //}

    // public boolean hasConeInIntake() {
    //     return !m_intakeSensorLeft.get() != !m_intakeSensorRight.get();
    // }

    // public boolean hasCubeInIntake() {
    //     return !m_intakeSensorRight.get();
    //}

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("intake", "Intake Angle", () -> getAngle(), 0, 0);
        NAR_Shuffleboard.addData("intake", "Pivot Velocity", () -> m_intakePivot.getSelectedSensorVelocity(), 2, 0);
        NAR_Shuffleboard.addData("intake", "Angle Setpoint", () -> getSetpoint(), 3, 0);
        NAR_Shuffleboard.addData("intake", "Roller Velocity", () -> m_intakeRollers.getSelectedSensorVelocity() / 4096,
                4, 0);
        setpoint = NAR_Shuffleboard.debug("intake", "setpoint", 0, 1,2);
        NAR_Shuffleboard.addData("intake", "IsEnabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData("intake", "atSetpoint", ()-> getController().atSetpoint(), 1, 1);
        power = NAR_Shuffleboard.debug("intake", "power", 0.1, 1, 4);
        kF = NAR_Shuffleboard.debug("intake", "kF", IntakeConstants.kF, 2, 2);

        NAR_Shuffleboard.addComplex("intake", "PID Controller", m_controller, 0, 2);
    }
}
