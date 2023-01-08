package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motor.NAR_Motor;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class Manipulator extends SubsystemBase {
    private NAR_TalonFX manipulator_motor ;
    private static Manipulator instance;

    public Manipulator(){
        manipulator_motor  = new NAR_TalonFX( MANIPULATOR_MOTOR_ID, NAR_Motor.PRO_775);
        manipulator_motor.setNeutralMode(NeutralMode.Brake);
        
    }
    public static Manipulator getInstance() {
        if(instance == null){
            instance = new Manipulator() ;  
        }
        return instance;
    }
    public void openClaw(){
        manipulator_motor.set(-MANIPULATOR_MOTOR_SPEED_PERCENT);
    }
    public void closeClaw(){
        manipulator_motor.set(MANIPULATOR_MOTOR_SPEED_PERCENT);
    }
    public void stop(){
        manipulator_motor.set(0);
        // manipulator_motor.neutralOutput();
        
    }
    public void resetEncoder(){
        manipulator_motor.setEncoderPosition(0);
    }
    public double getCurrentTicks(){
        return manipulator_motor.getSelectedSensorPosition();
    }
}
