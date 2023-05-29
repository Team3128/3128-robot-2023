package frc.team3128.common.hardware.motorcontroller;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.team3128.Robot;
import frc.team3128.common.utility.NAR_Shuffleboard;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

/**
 * Team 3128's streamlined {@link CANSparkMax} class.
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public class NAR_CANSparkMax extends CANSparkMax {

	public enum EncoderType {
		Relative,
		Absolute
	}
	
	private double kP, kI, kD;
	private double prevValue = 0;
	private double prevFF = 0;
	private ControlType prevControlType = ControlType.kDutyCycle;
	private EncoderType encoderType;
	private SparkMaxRelativeEncoder relativeEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;
	private SparkMaxPIDController controller;

	/**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 * @param type The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 * @param kP The proportional coefficient of the on board PIDController.
	 * @param kI The integral coefficient of the on board PIDController.
   	 * @param kD The derivative coefficient of the on board PIDController.
	 */
	 public NAR_CANSparkMax(int deviceNumber, EncoderType encoderType, MotorType type, double kP, double kI, double kD) {
		super(deviceNumber, type);

		setCANTimeout(10);
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

		controller = getPIDController();
		controller.setOutputRange(-1, 1);
		controller.setP(kP);
		controller.setI(kI);
		controller.setD(kD);
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		
		controller.setFeedbackDevice(encoderType == EncoderType.Relative ? relativeEncoder : absoluteEncoder);
	}

	/**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 * @param encoderType The type of encoder used, ie. relative build in encoder or absolute encoder
	 * 		connected by an adapter.
	 * @param type The motor type connected to the controller. Brushless motor wires must be connected
	 *     	to their matching colors and the hall sensor must be plugged in. Brushed motors must be
	 *     	connected to the Red and Black terminals only.
	 */
	public NAR_CANSparkMax(int deviceNumber, EncoderType encoderType, MotorType type) {
		this(deviceNumber, encoderType, type, 0, 0, 0);
	}

	/**
	 * Create a new object to control a SPARK MAX motor
	 *
	 * @param deviceNumber The device ID.
	 */
	public NAR_CANSparkMax(int deviceNumber) {
		this(deviceNumber, EncoderType.Relative, MotorType.kBrushless);
	}

	/**
	 * Adds a PID tuning setup to a specific shuffleboard tab. Editing a value on the tab 
	 * will automatically update the value on the controller.
	 * @param tabName The title of the tab to select.
	 * @param prefix String added before each name of PID widgets.
	 * @param column Column to add the PID widgets to.
	 */
	public void initShuffleboard(String tabName, String prefix, int column) {
		DoubleSupplier proportional = NAR_Shuffleboard.debug(tabName, prefix + "_kP", kP, column, 0);
		DoubleSupplier integral = NAR_Shuffleboard.debug(tabName, prefix + "_kI", kI, column, 1);
		DoubleSupplier derivative = NAR_Shuffleboard.debug(tabName, prefix + "_kD", kD, column, 2);
		Robot.getInstance().addPeriodic(()-> {
			if (proportional.getAsDouble() == kP) {
				kP = proportional.getAsDouble();
				controller.setP(kP);
			}
		}, 0.200);
		Robot.getInstance().addPeriodic(()-> {
			if (integral.getAsDouble() == kI) {
				kI = proportional.getAsDouble();
				controller.setI(kI);
			}
		}, 0.200);
		Robot.getInstance().addPeriodic(()-> {
			if (derivative.getAsDouble() == kD) {
				kD = derivative.getAsDouble();
				controller.setD(kD);
			}
		}, 0.200);
	}

	/**
	 * Set the controller reference value based on basic duty cycle control
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1.
	 */
	@Override
	public void set(double outputValue) {
		set(outputValue, ControlType.kDutyCycle);
	}

	/**
	 * Set the controller reference value based on the selected control mode.
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
	 *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
	 *     (Amps).
	 * @param controlType Is the {@link ControlType} to override with
	 */
	public void set(double outputValue, CANSparkMax.ControlType controlType) {
		set(outputValue, controlType, 0);
	}

	/**
	 * Set the controller reference value based on the selected control mode.
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
	 *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
	 *     (Amps).
	 * @param controlType Is the {@link ControlType} to override with
	 * @param arbFeedforward A value from which is represented in voltage applied to the motor after
	 *     the result of the specified control mode. The units for the parameter is Volts. This value
	 *     is set after the control mode, but before any current limits or ramp rates.
	 */
	public void set(double outputValue, CANSparkMax.ControlType controlType, double arbFeedforward) {
		if (outputValue != prevValue || controlType != prevControlType || arbFeedforward != prevFF) {
			controller.setReference(outputValue, controlType, 0, arbFeedforward);
			prevValue = outputValue;
			prevControlType = controlType;
			prevFF = arbFeedforward;
		}
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to change the default rates.
	 * 
	 * <p><b>Status 1</b>: Applied Output, Faults, Sticky Faults, isFollower
	 * <p><b>Status 2</b>: Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
	 * <p><b>Status 3</b>: Motor Position
	 * <p><b>Status 4</b>: Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
	 * <p><b>Status 5</b>: Alternate Encoder Velocity
	 * <p><b>Status 6</b>: Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
	 * <p><b>Status 7</b>: Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency
	 * 
	 * @param frame Which type of {@link PeriodicFrame} to change the period of
	 * @param periodMs Period in ms for the given frame.
	 * @return {@link REVLibError#kOk} if successful
	 */
	@Override
	public REVLibError setPeriodicFramePeriod(PeriodicFrame frame, int periodMs) {
		return super.setPeriodicFramePeriod(frame, periodMs);
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's default rates.
	 *
	 * <p>Defaults: Status0 - 10ms Status1 - 20ms Status2 - 20ms Status3 - 255ms Status4 - 255ms Status5
	 * - 255ms Status6 - 255ms
	 */
	public void setDefaultStatusFrames() {
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, MotorControllerConstants.ULTRA_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, MotorControllerConstants.HIGH_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, MotorControllerConstants.HIGH_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus3, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus4, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus5, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus6, MotorControllerConstants.LOW_PRIORITY);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * <p>Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
	 *
	 * <p>Following anything other than a CAN SPARK MAX is not officially supported.
	 * 
	 * <p>Configures status frames to minimize CAN utilization.
	 *
	 * @param motor The motor controller to follow.
	 * @param invert Set the follower to output opposite of the leader
	 * @return {@link REVLibError#kOk} if successful
	 */
	@Override
	public REVLibError follow(final CANSparkMax motor, boolean invert) {
		setPeriodicFramePeriod(PeriodicFrame.kStatus0, MotorControllerConstants.MEDIUM_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus1, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus2, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus3, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus4, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus5, MotorControllerConstants.LOW_PRIORITY);
		setPeriodicFramePeriod(PeriodicFrame.kStatus6, MotorControllerConstants.LOW_PRIORITY);
		return super.follow(motor);
	}

	/**
	 * Enables continuous input.
	 *
	 * <p>Rather then using the max and min input range as constraints, the motor considers them to be the
	 * same point and automatically calculates the shortest route to the setpoint.
	 *
	 * @param minInput The minimum value expected from the input.
	 * @param maxInput The maximum value expected from the input.
	 */
	public void enableContinuousInput(double minInput, double maxInput) {
		enableContinuousInput(minInput, maxInput, 1);
	}

	/**
	 * Enables continuous input.
	 *
	 * <p>Rather then using the max and min input range as constraints, it considers them to be the
	 * same point and automatically calculates the shortest route to the setpoint.
	 *
	 * @param minInput The minimum value expected from the input.
	 * @param maxInput The maximum value expected from the input.
	 * @param factor The conversion factor to multiply the inputs by.
	 */
	public void enableContinuousInput(double minInput, double maxInput, double factor) {
		controller.setPositionPIDWrappingEnabled(true);
		controller.setPositionPIDWrappingMinInput(minInput * factor);
		controller.setPositionPIDWrappingMaxInput(maxInput * factor);
	}

	/**
	 * Set the conversion factor for position of the encoder. Multiplied by the native output units to
	 * give you position.
	 *
	 * @param factor The conversion factor to multiply the native units by
	 */
	public void setPositionConversionFactor(double factor) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setPositionConversionFactor(factor);
			return;
		}
		absoluteEncoder.setPositionConversionFactor(factor);
	}

	/**
	 * Set the conversion factor for velocity of the encoder. Multiplied by the native output units to
	 * give you velocity
	 *
	 * @param factor The conversion factor to multiply the native units by
	 */
	public void setVelocityConversionFactor(double factor) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setVelocityConversionFactor(factor);
			return;
		}
		absoluteEncoder.setVelocityConversionFactor(factor);
	}

	/**
	 * Set the position of the encoder. By default the units are 'rotations' and can be changed by a
	 * scale factor using setPositionConversionFactor().
	 *
	 * @param position Number of rotations of the motor
	 */
	public void setSelectedSensorPosition(double position) {
		if (encoderType == EncoderType.Relative) {
			relativeEncoder.setPosition(position);
			return;
		}
	}

	/**
	 * Get the position of the motor. This returns the native units of 'rotations' by default, and can
	 * be changed by a scale factor using setPositionConversionFactor().
	 *
	 * @return Number of rotations of the motor
	 */
	public double getSelectedSensorPosition() {
		return encoderType == EncoderType.Relative ? relativeEncoder.getPosition() : absoluteEncoder.getPosition();
	}

	/**
	 * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
	 * changed by a scale factor using setVelocityConversionFactor().
	 *
	 * @return Number the RPM of the motor
	 */
	public double getSelectedSensorVelocity() {
		return encoderType == EncoderType.Relative ? relativeEncoder.getVelocity() : absoluteEncoder.getVelocity();
	}

	/**
	 * Get the output of the motor. This returns the output in volts.
	 * 
	 * @return Volts applied to motor
	 */
	public double getMotorOutputVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}
}
