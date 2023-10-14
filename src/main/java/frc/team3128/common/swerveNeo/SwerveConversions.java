package frc.team3128.common.swerveNeo;

public class SwerveConversions {
    
    /**
     * @param rotations Rotations
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        return rotations * 360.0 / gearRatio;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Rotations
     */
    public static double degreesToRotations(double degrees, double gearRatio) {
        double rotations = degrees / 360.0 * gearRatio;
        return rotations;
    }

    /**
     * @param RPM rotations per minute of the motor
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return RPM of Mechanism
     */
    public static double motorRPMtoRPM(double RPM, double gearRatio) {
        double motorRPM = RPM;
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return Falcon Velocity Counts
     */
    public static double RPMToMotorRPM(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return motorRPM;
    }

    /**
     * @param RPM RPM of motor
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return Velocity MPS
     */
    public static double RPMToMPS(double RPM, double circumference, double gearRatio){
        double wheelRPM = motorRPMtoRPM(RPM, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return RPM of motor
     */
    public static double MPSToRPM(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToMotorRPM(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param rotations rotations of motor
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and mechanism
     * @return Meters traveled
     */
    public static double falconToMeters(double rotations, double circumference, double gearRatio){
        double mechRotations = rotations / gearRatio;
        return mechRotations * circumference;
    }

    /**
     * @param meters Meters traveled
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and Mechanism
     * @return rotations of motor
     */
    public static double metersToFalcon(double meters, double circumference, double gearRatio){
        double mechRotations = meters / circumference;
        double rotations = mechRotations * gearRatio;
        return rotations; 
    }

}
