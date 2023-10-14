package frc.team3128.common.hardware.motorcontroller;

/**
 * Constants for motor control / conversion. Should not be changed.
 */
public class MotorControllerConstants {

    public static final double FALCON_ENCODER_RESOLUTION = 2048; // CPR
    public static final double RPM_TO_FALCON = FALCON_ENCODER_RESOLUTION / 600; //Nu/100ms
    public static final double MAG_ENCODER_RESOLUTION = 4096; // CPR
    public static final double TALONSRX_ENCODER_RESOLUTION = 4096;

    public static final double SPARKMAX_ENCODER_RESOLUTION = 42; // CPR
    public static final double SPARKMAX_RPM_TO_NUpS = SPARKMAX_ENCODER_RESOLUTION / 60; // rotations/min -> counts/sec

    public static final int LOW_PRIORITY = 255;
    public static final int MEDIUM_PRIORITY = 45;
    public static final int HIGH_PRIORITY = 20;
    public static final int ULTRA_PRIORITY = 10;
}
