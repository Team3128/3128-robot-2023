package frc.team3128.subsystems;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.utility.NAR_Shuffleboard;

/**
 * A subsystem based off of {@link ProfiledPIDSubsystem} 
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public abstract class NAR_ProfiledPIDSubsystem extends SubsystemBase {
    protected final ProfiledPIDController m_controller;
    protected boolean m_enabled;
    private DoubleSupplier kS, kV, kG;
    private DoubleFunction<Double> kG_Function;
    private BooleanSupplier debug;
    private DoubleSupplier setpoint;

    /**
     * Creates a new ProfiledPIDSubsystem.
     *
     * @param controller the ProfiledPIDController to use
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     * @param kG_Function function in which kG is passed through
     */
    public NAR_ProfiledPIDSubsystem(ProfiledPIDController controller, double kS, double kV, double kG, DoubleFunction<Double> kG_Function) {
        m_controller = controller;
        this.kG_Function = kG_Function;
        initShuffleboard(kS, kV, kG);
    }

    /**
     * Creates a new ProfiledPIDSubsystem.
     *
     * @param controller the ProfiledPIDController to use
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kG The gravity gain.
     */
    public NAR_ProfiledPIDSubsystem(ProfiledPIDController controller, double kS, double kV, double kG) {
        this(controller, kS, kV, kG, KG -> KG);
    }

    /**
     * Creates a new ProfiledPIDSubsystem.
     *
     * @param controller the ProfiledPIDController to use
     */
    public NAR_ProfiledPIDSubsystem(ProfiledPIDController controller) {
        this(controller, 0, 0, 0);
    }

    @Override
    public void periodic() {
        if (m_enabled) {
          double output = m_controller.calculate(getMeasurement());
          output += kS.getAsDouble() * Math.signum(getSetpoint().velocity);
          output += kV.getAsDouble() * getSetpoint().velocity;
          output += kG_Function.apply(kG.getAsDouble());
          useOutput(output, m_controller.getSetpoint());
        }
    }

    private void initShuffleboard(double kS, double kV, double kG) {
        NAR_Shuffleboard.addComplex(getName(), "PID_Controller", m_controller, 0, 0);

        NAR_Shuffleboard.addData(getName(), "Enabled", ()-> isEnabled(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Measurement", ()-> getMeasurement(), 1, 1);
        NAR_Shuffleboard.addData(getName(), "Goal", ()-> getGoal().position, 1, 2);

        var debugEntry = NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 2, 0).withWidget("Toggle Button");
        debug = ()-> debugEntry.getEntry().getBoolean(false);
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 2, 1);
        setpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 2,2);

        this.kS = NAR_Shuffleboard.debug(getName(), "kS", kS, 3, 0);
        this.kV = NAR_Shuffleboard.debug(getName(), "kV", kV, 3, 1);
        this.kG = NAR_Shuffleboard.debug(getName(), "kG", kG, 3, 2);
  }

    public ProfiledPIDController getController() {
        return m_controller;
    }

    /**
     * Sets the goal state for the subsystem.
     *
     * @param goal The goal state for the subsystem's motion profile.
     */
    public void startPID(TrapezoidProfile.State goal) {
        m_controller.setGoal(goal);
        enable();
    }

    /**
     * Sets the goal state for the subsystem. Goal velocity assumed to be zero.
     *
     * @param goal The goal position for the subsystem's motion profile.
     */
    public void startPID(double goal) {
        startPID(new TrapezoidProfile.State(debug.getAsBoolean() ? setpoint.getAsDouble() : goal, 0));
    }

    /**
     * Returns the current setpoint of the subsystem.
     *
     * @return The current setpoint of the subsystem.
     */
    public TrapezoidProfile.State getSetpoint() {
        return m_controller.getSetpoint();
    }

    /**
     * Returns the goal state for the subsystem
     *
     * @return The goal position for the subsystem's motion profile.
     */
    public TrapezoidProfile.State getGoal() {
        return m_controller.getGoal();
    }
    
    /**
     * Returns if at the current setpoint of the subsystem.
     *
     * @return if subsystem is at current setpoint.
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Returns if at the goal state for the subsystem
     *
     * @return if subsystem is at goal state
     */
    public boolean atGoal() {
        return m_controller.atGoal();
    }

    /**
     * Uses the output from the ProfiledPIDController.
     *
     * @param output the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController, for feedforward
     */
    protected abstract void useOutput(double output, State setpoint);

    /**
     * Returns the measurement of the process variable used by the ProfiledPIDController.
     *
     * @return the measurement of the process variable
     */
    protected abstract double getMeasurement();

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        m_enabled = true;
        m_controller.reset(getMeasurement());
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        m_enabled = false;
        useOutput(0, new State());
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return m_enabled;
    }
}

