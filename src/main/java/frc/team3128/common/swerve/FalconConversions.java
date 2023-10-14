package frc.team3128.common.swerve;

public class FalconConversions {

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        //return counts * (360.0 / (gearRatio * 2048.0));
        return counts * 360.0 / gearRatio;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        // double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        // return ticks;
        double rotations = degrees / 360.0 * gearRatio;
        return rotations;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        // double motorRPM = velocityCounts * (600.0 / 2048.0);        
        // double mechRPM = motorRPM / gearRatio;
        // return mechRPM;
        double motorRPM = velocityCounts;
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        // double motorRPM = RPM * gearRatio;
        // double sensorCounts = motorRPM * (2048.0 / 600.0);
        // return sensorCounts;
        double motorRPM = RPM * gearRatio;
        return motorRPM;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Velocity MPS
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param distancecount Falcon Distance Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Meters traveled
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
        // return (positionCounts / 2048 * gearRatio * circumference); // I really hate units - mika
        //return positionCounts * (circumference / (gearRatio * 2048.0)); //Me too <3 - Mason
        double rotations = positionCounts;
        double mechRotations = rotations / gearRatio;
        return mechRotations * circumference;
        // in theory 2048 = units/rev (falcon number)
    }

    /**
     * @param meters Meters traveled
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Distance Counts
     */
    public static double metersToFalcon(double meters, double circumference, double gearRatio){
        //return meters / (circumference / (gearRatio * 2048.0));
        double mechRotations = meters / circumference;
        double rotations = mechRotations * gearRatio;
        return rotations; 
    }

}