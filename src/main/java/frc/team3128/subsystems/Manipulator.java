package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motor.NAR_Motor;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class Manipulator extends PIDSubsystem {

    public enum ManipulatorState {

        OPEN(5), CLOSED_CUBE(374), CLOSED_CONE(350);

        private double tickValue;
        ManipulatorState(double tickValue) {
            this.tickValue = tickValue;
        }

        public double getTickValue() {
            return tickValue;
        }
    }

    private NAR_TalonFX manipulator_motor;
    private static Manipulator instance;
    private int plateauCount = 0;

    public Manipulator(){
        super(new PIDController(K_P, K_I, K_D));
        manipulator_motor  = new NAR_TalonFX( MANIPULATOR_MOTOR_ID );
        manipulator_motor.setNeutralMode(NeutralMode.Brake);
        
    }
    public static Manipulator getInstance() {
        if(instance == null){
            instance = new Manipulator() ;  
        }
        return instance;
    }
    
    public void stop(){
        manipulator_motor.set(0);
        // manipulator_motor.neutralOutput();
        
    }

    public void beginPID(ManipulatorState state) {
        beginPID(state.getTickValue());
    }

    public void beginPID(double desiredTickValue) {
        plateauCount = 0;
        enable();
        setSetpoint(desiredTickValue);
        getController().setTolerance(desiredTickValue * 0.05);
    }

    public void resetEncoder(){
        manipulator_motor.setEncoderPosition(0);
    }
    public double getCurrentTicks(){
        return manipulator_motor.getSelectedSensorPosition();
    }

    @Override
    protected void useOutput(double output, double setpoint) {

        double ff = K_F*setpoint;
        double voltageOutput = output+ff;


        if (getController().atSetpoint() && (setpoint != 0)) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }

        if (setpoint == 0) voltageOutput = 0;

        manipulator_motor.set(MathUtil.clamp(voltageOutput / 12.0, -1, 1));
        
    }
    @Override
    protected double getMeasurement() {
        return getCurrentTicks();
    }

    public boolean isReady() {
        return (plateauCount >= PLATEAU_COUNT) && (getSetpoint() != 0);
    }

    public void resetPlateauCount() {
        plateauCount = 0;
    }


}
