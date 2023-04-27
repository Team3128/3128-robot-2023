package frc.team3128.common.hardware.motorcontroller;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;

public class NAR_TalonFX extends WPI_TalonFX {

    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private TalonFXSimCollection motorSim;

	/**	 
	 * @param deviceNumber device id
	 */
	public NAR_TalonFX(int deviceNumber, String canBus) {
		super(deviceNumber, canBus);

		if(RobotBase.isSimulation()){
			motorSim = getTalonFXSimCollection();
		}

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);
	}

	public NAR_TalonFX(int deviceNumber) {
		this(deviceNumber, "");
	}

	@Override
	public void set(double speed) {
		set(ControlMode.PercentOutput, speed);
	}
  
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
			prevControlMode = controlMode;
		}
	}
	
	public double getSetpoint() {
		return prevValue;
	}

	public ControlMode getControlMode() {
		return prevControlMode;
	}

	// getInverted() stuff should only be temporary
	public void setSimPosition(double pos) {
		if(super.getInverted()) {
			pos *= -1;
		}
		motorSim.setIntegratedSensorRawPosition((int)pos);
	}

	// getInverted() stuff should only be temporary
	public void setSimVelocity(double vel) {
		if(super.getInverted()) {
			vel *= -1;
		}
		motorSim.setIntegratedSensorVelocity((int)(vel/10)); // convert nu/s to nu/100ms
	}

	@Override
	public ErrorCode setSelectedSensorPosition(double n) {
		return super.setSelectedSensorPosition(n * MotorControllerConstants.FALCON_ENCODER_RESOLUTION); //Rotations to nu
	}

	@Override
	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity() * 600 / MotorControllerConstants.FALCON_ENCODER_RESOLUTION; // convert nu/100ms to rpm
	}

	@Override
	public double getSelectedSensorPosition() {
		return super.getSelectedSensorPosition() / MotorControllerConstants.FALCON_ENCODER_RESOLUTION; // convert nu to rotations
	}
}