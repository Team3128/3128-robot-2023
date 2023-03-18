package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;
import static frc.team3128.Constants.ManipulatorConstants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Manipulator extends SubsystemBase {

    private NAR_TalonSRX m_roller;

    private static Manipulator instance;

    public static boolean CONE = true;

    public Manipulator(){
        configMotor();
    }

    public static Manipulator getInstance() {
        if (instance == null){
            instance = new Manipulator();  
        }
        
        return instance;
    }

    @Override
    public void periodic() {
        if (Math.abs(getCurrent()) > CUBE_CURRENT_THRESHOLD + 20)
            enableRollersStall();
    }

    public void configMotor(){
        m_roller = new NAR_TalonSRX(ROLLER_MOTOR_ID);
        m_roller.setInverted(true);
        m_roller.setNeutralMode(NeutralMode.Brake);
        m_roller.enableVoltageCompensation(true);
    }

    public void setRollerPower(double power){
        m_roller.set(power);
    }

    public void enableRollersForward(){
        m_roller.set(ROLLER_POWER);
    }

    public void enableRollersReverse(){
        m_roller.set(-ROLLER_POWER);
    }

    public void stopRoller(){
        m_roller.set(0);
    }

    public double getCurrent(){
        return m_roller.getStatorCurrent();
    }

    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > (CONE ? CONE_CURRENT_THRESHOLD : CUBE_CURRENT_THRESHOLD);
    }

    public void intake(boolean cone) {
        CONE = cone;
        if (cone) enableRollersReverse();
        else enableRollersForward();
    }    

    public void outtake(){
        if (CONE) enableRollersForward();
        else enableRollersReverse();
    }

    public void enableRollersStall() {
        setRollerPower(CONE ? -STALL_POWER : STALL_POWER);
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Manipulator", "Manip current", () -> getCurrent(), 0, 1);
        NAR_Shuffleboard.addData("Manipulator", "get", () -> m_roller.getMotorOutputPercent(), 0, 3);
        NAR_Shuffleboard.addData("Manipulator", "ObjectPresent", ()-> hasObjectPresent(), 1, 1);
    }
}
