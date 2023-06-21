package frc.team3128.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"rawtypes", "unchecked"})
public abstract class NAR_StateSpaceSubsystem extends SubsystemBase{

    public enum System {
        POSITION,
        VELOCITY
    }

    protected final LinearSystemLoop loop;
    protected final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State lastProfiledReference;
    private System system;
    protected boolean m_enabled;
    private double tolerance;
    private double setpoint;

    /**
     * Create a new NAR_StateSpaceSubsystem object
     * 
     * @param loop LinearSystemLoop representing the physical subsystem
     * @param constraints Trapezoid constraints with max velocity and accelaration
     * @param system Type of Linear System setpoint: (Position/Velocity)
     */
    public NAR_StateSpaceSubsystem(LinearSystemLoop loop, TrapezoidProfile.Constraints constraints, System system) {
        this.loop = loop;
        this.constraints = constraints;
        this.system = system;
        reset();
    }

    /**
     * Create a new NAR_StateSpaceSubsystem object
     * 
     * @param loop LinearSystemLoop representing the physical subsystem
     * @param system Type of Linear System (Velocity/Position)
     */
    public NAR_StateSpaceSubsystem(LinearSystemLoop loop, System system) {
        this(loop, new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY), system);
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void startSSC(double setpoint) {
        this.setpoint = setpoint;
        enable();
    }

    /** Enables State Space control. Resets the controller. */
    public void enable(){
        reset();
        m_enabled = true;
    }

    /** Disables State Space control. Sets output to zero. */
    public void disable(){
        m_enabled = false;
        useOutput(0);
    }

    /**
     * Sets the setpoint for the subsystem.
     *
     * @param setpoint the setpoint for the subsystem
     */
    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    /**
     * Gets the setpoint for the subsystem.
     *
     * @return the setpoint for the subsystem
     */
    public double getSetpoint() {
        return setpoint;
    }

    /** Set the tolerance for the subsystem */
    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    /**
     * Returns whether the subsystem is at setpoint
     *
     * @return true if at setpoint and false if not
     */
    public boolean atSetpoint() {
        double measurement = system == System.POSITION ? setpoint - getPosition() : setpoint - getVelocity();
        return Math.abs(measurement) < tolerance;
    }

    @Override
    public void periodic() {
        if(m_enabled){
            double output = calculate(setpoint);
            useOutput(output);
        }
    }

    /**
     * Uses the output from the LinearSystemLoop.
     *
     * @param output the output of the LinearSystemLoop
     */
    protected abstract void useOutput(double output);

    /**
     * Returns the current position of the linear system.
     * Not needed if the system type is velocity based.
     */
    protected abstract double getPosition();

    /**
     * Returns the current velocity of the linear system
     */
    protected abstract double getVelocity();
    
    /**
     * Calculates the desired output voltage for the motor(s)
     * @param desiredSetpoint the setpoint for the linearSystem
     * @return output in voltage
     */
    public double calculate(double desiredSetpoint) {
        if (system == System.POSITION) {
            //Goal represented by the end position and a velocity of 0
            TrapezoidProfile.State setpoint = new TrapezoidProfile.State(desiredSetpoint, 0.0); 
            //Estimate the position and velocity of the system using the previous state 20ms ago
            lastProfiledReference = (new TrapezoidProfile(constraints, setpoint, lastProfiledReference)).calculate(0.020);
            loop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
        }
        else {
            //Set the desired velocity
            loop.setNextR(desiredSetpoint);
        }
        //Update the LinearSystemLoop with encoder measurement
        loop.correct(VecBuilder.fill(system == System.POSITION ? getPosition() : getVelocity()));
        //Predict the state of the linear system 20ms into the future
        loop.predict(0.020);
        //Get the voltage applied to the motors in the linear system in the model
        double nextVoltage = loop.getU(0);
        return MathUtil.clamp(nextVoltage, -12.0, 12.0);
    }

    /**
     * Resets the linear system to the current position/velocity.
     */
    protected void reset(){
        if (system == System.POSITION) {
            loop.reset(VecBuilder.fill(getPosition(), getVelocity()));
            lastProfiledReference = new TrapezoidProfile.State(getPosition(), getVelocity());
        }
        if (system == System.VELOCITY) {
            loop.reset(VecBuilder.fill(getVelocity()));
        }
    }
}