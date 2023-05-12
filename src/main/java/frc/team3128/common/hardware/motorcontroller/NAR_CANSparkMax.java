package frc.team3128.common.hardware.motorcontroller;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class NAR_CANSparkMax extends CANSparkMax {

	public enum EncoderType {
		Relative,
		Absolute
	}
	
	private double prevValue = 0;
	private ControlType prevControlType = ControlType.kDutyCycle;
	private double prevFeedForward = 0;
	private EncoderType encoderType;
	private SparkMaxRelativeEncoder relativeEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;
	private SparkMaxPIDController controller;
	private SimDeviceSim encoderSim;
	private SimDouble encoderSimVel;

	/**
	 * 
	 * @param deviceNumber device id
	 * @param type         kBrushed(0) for brushed motor, kBrushless(1) for brushless motor
	 */
	 public NAR_CANSparkMax(int deviceNumber, EncoderType encoderType, MotorType type, double kP, double kI, double kD) {
		super(deviceNumber, type);

		restoreFactoryDefaults(); // Reset config parameters, unfollow other motor controllers
		enableVoltageCompensation(12);

		this.encoderType = encoderType;

		if (encoderType == EncoderType.Relative) {
			relativeEncoder = (SparkMaxRelativeEncoder) getEncoder();
		}
		else {
			absoluteEncoder = getAbsoluteEncoder(Type.kDutyCycle);
			absoluteEncoder.setVelocityConversionFactor(60);
		}

		// encoder.setPositionConversionFactor(MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION); // convert rotations to encoder ticks
		// encoder.setVelocityConversionFactor(MotorControllerConstants.SPARKMAX_RPM_TO_NUpS); // convert rpm to nu/s
		controller = getPIDController();
		controller.setP(kP);
		controller.setI(kI);
		controller.setD(kD);

		if(RobotBase.isSimulation()){
			encoderSim = new SimDeviceSim("CANSparkMax[" + this.getDeviceId() + "] - RelativeEncoder");
			encoderSimVel = encoderSim.getDouble("Velocity");
		}
	}

	public NAR_CANSparkMax(int deviceNumber, EncoderType encoderType, MotorType type) {
		this(deviceNumber, encoderType, type, 0, 0, 0);
	}

	public NAR_CANSparkMax(int deviceNumber) {
		this(deviceNumber, EncoderType.Relative, MotorType.kBrushless);
	}

	@Override
	public void set(double outputValue) {
		set(outputValue, ControlType.kDutyCycle);
	}

	public void set(double outputValue, ControlType controlType) {
		set(outputValue, controlType, 0);
	}

	public void set(double outputValue, ControlType controlType, double arbFeedforward) {
		if (outputValue != prevValue || prevControlType != controlType || arbFeedforward != prevFeedForward) {
			controller.setReference(outputValue, controlType, 0, arbFeedforward);
			prevValue = outputValue;
			prevControlType = controlType;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

	//Default Unit: Rotations
	public double getSelectedSensorPosition() {
		return encoderType == EncoderType.Relative ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
	}

	//Default Unit: Rotations per minute
	public double getSelectedSensorVelocity() {
		return encoderType == EncoderType.Relative ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
	}

	public void setPositionConversionFactor(double factor) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setPositionConversionFactor(factor);
			return;
		}
		absoluteEncoder.setPositionConversionFactor(factor);
	}

	public void setVelocityConversionFactor(double factor) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setVelocityConversionFactor(factor);
			return;
		}
		absoluteEncoder.setVelocityConversionFactor(factor);
	}

	public double getMotorOutputVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public void setEncoderPosition(double encPos) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setPosition(encPos);
			return;
		}
	}

	public void setSimPosition(double pos) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setPosition(pos);
			return;
		}
	}

	public void setSimVelocity(double vel) {
		encoderSimVel.set(vel);
	}
}
