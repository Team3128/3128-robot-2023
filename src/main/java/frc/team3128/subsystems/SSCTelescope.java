package frc.team3128.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import static frc.team3128.Constants.TelescopeConstants.*;
import com.revrobotics.CANSparkMax.IdleMode;
public class SSCTelescope extends NAR_StateSpaceSubsystem{
    public static SSCTelescope instance;
    private static NAR_CANSparkMax motor;
    private DoubleSolenoid pisBreak;
    private static double elevatorMass = 0;
    private static double elevatorSpoolRadius = 0;
    private static double gearRatio = 0;
    private static double maxVelocity = 0;
    private static double maxAcceleration = 0;
    private static double modelPositionAccuracy = 0.1;
    private static double modelVelocityAccuracy = 0.1;
    private static double modelPositionTolerance = Units.inchesToMeters(1.0);
    private static double modelVelocityTolerance = Double.POSITIVE_INFINITY;
    private final static TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    private final static LinearSystem<N2, N1, N1> plant =
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), elevatorMass, elevatorSpoolRadius, gearRatio);
    private final static KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            plant,
            VecBuilder.fill(modelPositionAccuracy, modelVelocityAccuracy),
            VecBuilder.fill(0.001),
            0.020 /* sec */);
    private final static LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            plant,
            VecBuilder.fill(modelPositionTolerance, modelVelocityTolerance),
            VecBuilder.fill(12.0),
            0.020);
    private final static LinearSystemLoop<N2, N1, N1> loop = new LinearSystemLoop<N2, N1, N1>(
        plant,
        controller,
        filter,
        12.0,
        0.020
    );
    public SSCTelescope() {
        super(loop, constraints, NAR_StateSpaceSubsystem.System.POSITION);
        configMotors();
        configPneumatics();
        configEncoder();
        reset();
    }
    public static synchronized SSCTelescope getInstance() {
        if (instance == null)
            instance = new SSCTelescope();
        return instance;
    }
    private void configMotors() {
        motor = new NAR_CANSparkMax(TELE_MOTOR_ID);
        motor.setDefaultStatusFrames();
        motor.setSmartCurrentLimit(TELE_CURRENT_LIMIT);
        motor.enableVoltageCompensation(12.0);
        motor.setIdleMode(IdleMode.kBrake);
    }
    private void configPneumatics() {
        pisBreak = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL_ID, SOLENOID_BACKWARD_CHANNEL_ID);
        brake(true);
    }
    private void configEncoder() {
        motor.setPositionConversionFactor(ENC_CONV);
    }
    public void resetEncoders() {
        motor.setSelectedSensorPosition(0);
    }
    private void brake(boolean engaged) {
        pisBreak.set(engaged ? Value.kForward : Value.kReverse);
    }
    public void move(boolean forwards){
        move(0.1 * (forwards? 1: -1));
    }
    public void move(double speed){
        motor.set(speed);
    }
    public void stop() {
        disable();
        motor.set(0);
    }
    public void startSSC(double setpoint){
        brake(false);
        super.startSSC(MathUtil.clamp(setpoint, 0, 30));
    }
    @Override
    protected void useOutput(double output) {
        motor.setVoltage(output);
        if(atSetpoint()){
            disable();
            brake(true);
        }
    }
    @Override
    protected double getPosition() {
        return motor.getSelectedSensorPosition();
    }
    @Override
    protected double getVelocity() {
        return motor.getSelectedSensorVelocity();
    }
}