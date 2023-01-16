package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motor.NAR_Motor;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase {
    
    private DoubleSolenoid m_solenoid;
    //private NAR_TalonFX manipulator_motor ;
    private static Manipulator instance;

    public Manipulator(){
        m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, MANIPULATOR_SOLENOID_FORWARD_CHANNEL_ID, MANIPULATOR_SOLENOID_BACKWARD_CHANNEL_ID);
        m_solenoid.set(Value.kReverse);
        // manipulator_motor  = new NAR_TalonFX( MANIPULATOR_MOTOR_ID );
        // manipulator_motor.setNeutralMode(NeutralMode.Brake);
        
    }
    public static Manipulator getInstance() {
        if(instance == null){
            instance = new Manipulator() ;  
        }
        return instance;
    }
    public void openClaw(){
        m_solenoid.set(Value.kReverse);
        //manipulator_motor.set(-MANIPULATOR_MOTOR_SPEED_PERCENT);
    }
    public void closeClaw(){
        m_solenoid.set(Value.kForward);
        //manipulator_motor.set(MANIPULATOR_MOTOR_SPEED_PERCENT);
    }

    public Value getClawState() {
        return m_solenoid.get();
    }
    
    public void toggleClaw() {
        if (getClawState() == Value.kForward) {
            openClaw();
        }
        else if (getClawState() == Value.kReverse || getClawState() == Value.kOff) {
            closeClaw();
        }
    }

    // public void stop(){
    //     manipulator_motor.set(0);
    //     // manipulator_motor.neutralOutput();
        
    // }
    // public void resetEncoder(){
    //     manipulator_motor.setEncoderPosition(0);
    // }
    // public double getCurrentTicks(){
    //     return manipulator_motor.getSelectedSensorPosition();
    // }
    
}
