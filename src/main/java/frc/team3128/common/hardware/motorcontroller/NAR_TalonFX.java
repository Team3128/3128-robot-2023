package frc.team3128.common.hardware.motorcontroller;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.team3128.Robot;
import frc.team3128.common.utility.NAR_Shuffleboard;

/**
 * Team 3128's streamlined {@link WPI_TalonFX} class.
 * @since 2023 CHARGED UP
 * @author Mason Lam
 */
public class NAR_TalonFX extends WPI_TalonFX {

	private double kP, kI, kD;
	private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;
	private double prevFF = 0;

	/**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 * @param canBus name of the CanBus, change when using Canivore
	 * @param kP The proportional coefficient of the on board PIDController.
	 * @param kI The integral coefficient of the on board PIDController.
   	 * @param kD The derivative coefficient of the on board PIDController.
	 */
	public NAR_TalonFX(int deviceNumber, String canBus, double kP, double kI, double kD) {
		super(deviceNumber, canBus);

		configVoltageCompSaturation(12, 10);
		enableVoltageCompensation(true);

		this.kP = kP;
		this.kI = kI;
		this.kD = kD;

		config_kP(0, kP);
		config_kI(0, kI);
		config_kD(0, kD);
	}

	/**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 * @param canBus name of the CanBus, change when using Canivore
	 */
	public NAR_TalonFX(int deviceNumber, String canBus) {
		this(deviceNumber, canBus, 0, 0, 0);
	}

	/**
	 * Create a new object to control a Falcon 500 motor
	 *
	 * @param deviceNumber The device ID.
	 */
	public NAR_TalonFX(int deviceNumber) {
		this(deviceNumber, "");
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
				config_kP(0, kP);
			}
		}, 0.200);
		Robot.getInstance().addPeriodic(()-> {
			if (integral.getAsDouble() == kI) {
				kI = proportional.getAsDouble();
				config_kI(0, kI);
			}
		}, 0.200);
		Robot.getInstance().addPeriodic(()-> {
			if (derivative.getAsDouble() == kD) {
				kD = derivative.getAsDouble();
				config_kD(0, kD);
			}
		}, 0.200);
	}

	/**
	 * Set the controller reference value based on percent output control
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1.
	 */
	@Override
	public void set(double speed) {
		set(speed, ControlMode.PercentOutput);
	}

	/**
	 * Set the controller reference value based on the selected control mode.
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
	 *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
	 *     (Amps).
	 * @param controlMode Is the {@link ControlMode} to override with
	 */
	public void set(double outputValue, ControlMode controlMode) {
		set(controlMode, outputValue, DemandType.Neutral, 0);
	}

	/**
	 * Set the controller reference value based on the selected control mode.
	 *
	 * @param outputValue The value to set depending on the control mode. For basic duty cycle control this
	 *     should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts) Velocity
	 *     Control: Velocity (RPM) Position Control: Position (Rotations) Current Control: Current
	 *     (Amps).
	 * @param controlMode Is the {@link ControlMode} to override with
	 * @param feedForward A value which is measured in motor output [-1,1] applied to the motor after
	 *     the result of the specified control mode.
	 */
	public void set(double outputValue, ControlMode controlMode, double feedForward) {
		if (controlMode == ControlMode.Position) {
			outputValue *= MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
		}
		if (controlMode == ControlMode.Velocity) {
			outputValue *= MotorControllerConstants.RPM_TO_FALCON;
		}
		if (outputValue != prevValue || controlMode != prevControlMode|| feedForward != prevFF) {
			super.set(controlMode, outputValue, DemandType.ArbitraryFeedForward, feedForward);
			prevValue = outputValue;
			prevControlMode = controlMode;
			prevFF = feedForward;
		}
	}

	/**
	 * Set the rate of transmission for status frames from the TalonFX
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to change the default rates.
	 * 
	 * <p><b>Status 1</b>: Applied Output, Faults information, Limit Switch Information
	 * <p><b>Status 2</b>: Selected Sensor Position, Selected Sensor Velocity, Brushed Current Measurement, Sticky Fault information
	 * <p><b>Status 3</b>: Quadrature information
	 * <p><b>Status 4</b>: Analog Input, Supply Battery Voltage, Controller Temperature
	 * <p><b>Status 8</b>: Pulse Width information
	 * <p><b>Status 10</b>: Motion profiling/Motion magic information
	 * <p><b>Status 12</b>: Selected Sensor Position (AUX PID), Selected Sensor Velocity (AUX PID)
	 * <p><b>Status 13</b>: PID0 Primary PID information
	 * <p><b>Status 14</b>: PID1 Auxillary PID information
	 * <p><b>Status 21</b>: Integrated Sensor Position (TalonFX), Integrated Sensor Velocity (TalonFX)
	 * 
	 * @param frame which {@link StatusFrameEnhanced} to be changed.
	 * @param periodMs Period in ms for the given frame.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs) {
		int timeoutMs = 0;
		return setStatusFramePeriod(frame, periodMs, timeoutMs);
	}

	/**
	 * Set the rate of transmission for stauts frames from the TalonFX
	 *
	 * <p>Each motor controller sends back status frames with different data at set rates. Use this
	 * function to set to team 3128's default rates.
	 *
	 * <p>Defaults: Status1 - 10ms Status2 - 20ms Status3 - 255ms Status4 - 255ms Status8 - 255ms Status10
	 * - 255ms Status12 - 255ms  Status13 - 255ms  Status14 - 255ms  Status21 - 255ms
	 */
	public void setDefaultStatusFrames() {
		setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MotorControllerConstants.ULTRA_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MotorControllerConstants.HIGH_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, MotorControllerConstants.LOW_PRIORITY);
		setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, MotorControllerConstants.LOW_PRIORITY);
	}

	/**
	 * Sets the sensor position to the given value.
	 *
	 * @param sensorPos Position to set for the selected sensor (in rotations).
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	@Override
	public ErrorCode setSelectedSensorPosition(double sensorPos) {
		return super.setSelectedSensorPosition(sensorPos * MotorControllerConstants.FALCON_ENCODER_RESOLUTION); //Rotations to nu
	}

	/**
	 * Get the selected sensor velocity.
	 *
	 * @return sensor velocity in rotations per minute (RPM)
	 */
	@Override
	public double getSelectedSensorVelocity() {
		return super.getSelectedSensorVelocity() * 600 / MotorControllerConstants.FALCON_ENCODER_RESOLUTION; // convert nu/100ms to rpm
	}

	/**
	 * Get the selected sensor position
	 *
	 * @return Position of selected sensor (in rotations).
	 */
	@Override
	public double getSelectedSensorPosition() {
		return super.getSelectedSensorPosition() / MotorControllerConstants.FALCON_ENCODER_RESOLUTION; // convert nu to rotations
	}
}