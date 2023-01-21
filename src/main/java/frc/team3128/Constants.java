package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.swerve.SecondOrderSwerveDriveKinematics;
import frc.team3128.common.swerve.SwerveModuleConstants;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import frc.team3128.common.utility.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class ConversionConstants {

        public static final double SPARK_VELOCITY_FACTOR = SPARKMAX_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm
    }

    public static class SwerveConstants {
        public static final int pigeonID = 30; 
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26); // Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(26); // Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4); // Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; // (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = (150.0 / 7.0); // 21.43; // (12.8 / 1.0); //12.8:1 

        public static final SecondOrderSwerveDriveKinematics swerveKinematics = new SecondOrderSwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); // laser was broken so offset

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final double TURN_TOLERANCE = 5;

        public static final double DRIVE_TOLERANCE = 0.05;

        public static final double translationKP = 3;
        public static final double translationKI = 0;
        public static final double translationKD = 0;

        /* Translation PID Values */
        public static final double distanceKP = 1;
        public static final double distanceKI = 0;
        public static final double distanceKD = 0;

        /* Rotation PID Values */
        public static final double alignKP = 0.05;
        public static final double alignKI = 0;
        public static final double alignKD = 0;

        /* Rotation PID Values */
        public static final double rotationKP = 3;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        /* Turning PID Values */
        public static final double turnKP = 0.1;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnKF = 0.1;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3; // 0.6; // citrus: 0.3
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;//1.476;//0.05; // citrus: 0.05 //sysid 2.9424
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // TODO: sysid this
        public static final double driveKS = 0.60094;//(0.49321 / 12);
        public static final double driveKV = 1.1559;//(2.4466 / 12);
        public static final double driveKA = 0.12348;//(0.22036 / 12);

         /* Angle Motor Characterization Values */

         //TODO: characterize I guess
         public static final double angleKS = (0.60094/12);//(0.49321 / 12);
         public static final double angleKV = (1.1559/12);//(2.4466 / 12);
         public static final double angleKA = (0.12348/12);//(0.22036 / 12);

        public static final double turnTolerance = 2;

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxWheelSpeed = 2; //4.5// 4.96824; // citrus: 4.5 //meters per second - 16.3 ft/sec
        public static final double maxAcceleration = 0.5;
        public static final double maxAngularVelocity = 0.5;//3; //11.5; // citrus: 10
        public static final double maxSpeed = 0.125;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 20;
            public static final double angleOffset = -157.763671875; // deg
            public static final String canBus = "drivetrain";
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final double angleOffset = 129.375; // deg
            public static final String canBus = "drivetrain";
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 22;
            public static final double angleOffset = -69.697265625; // deg
            public static final String canBus = "drivetrain"; 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 23;
            public static final double angleOffset = -54.31640625; // deg
            public static final String canBus = "rio";
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }
    }

    public static class DriveConstants {

        // public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 * Math.PI / FALCON_ENCODER_RESOLUTION;
        // public static final double DRIVE_NU_TO_METER = ENCODER_DISTANCE_PER_MARK / DRIVE_GEARING; // meters driven per encoder tick
        // public static final double DRIVE_NUp100MS_TO_MPS = DRIVE_NU_TO_METER * 10; // sensor units per 100 ms to m/s of drivetrain
        // public static final double MAX_DRIVE_VEL_NUp100MS = 6380 * FALCON_ENCODER_RESOLUTION / 60 / 10; // max angular velocity of drivetrain (encoder, not wheel) in sensor units per 100 ms - 6380 RPM * RESOLUTION nu/rot * 1 min/60s * 1s/(10*100ms)

    }

    // subsystem constants classes here

    public static class VisionConstants {

        public static final Camera SHOOTER = new Camera("Frog", true, 0, 0, 0,  new Transform2d(new Translation2d(Units.inchesToMeters(-12),0), Rotation2d.fromDegrees(0)));

        public static final double SCREEN_WIDTH = 320;
        public static final double SCREEN_HEIGHT = 240;
    
        public static final double HORIZONTAL_FOV = 59.6; //degrees
        public static final double VERTICAL_FOV = 45.7; //degrees

        public static final double TX_THRESHOLD = 3; // degrees

        public static final double ANGLE_THRESHOLD = 10; // degrees

        public static final double TARGET_AREA = 6.25 * 6.25; //inches

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(10,10,Units.degreesToRadians(0.1));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(1));

        public static final Pose2d[] SCORES = new Pose2d[]{
            new Pose2d(2.1,0.5,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,1.05,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,1.65,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,2.15,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,2.75,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,3.3,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,3.85,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,4.45,Rotation2d.fromDegrees(180)),
            new Pose2d(2.1,5,Rotation2d.fromDegrees(180))
        };

        public static final HashMap<Integer,Pose2d> APRIL_TAG_POS = new HashMap<Integer,Pose2d>();

        static {
            APRIL_TAG_POS.put(1, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(2, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(3, new Pose2d(
                new Translation2d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(4, new Pose2d(
                new Translation2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(180))
            );
            APRIL_TAG_POS.put(5, new Pose2d(
                new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(6, new Pose2d(
                new Translation2d( Units.inchesToMeters(40.45), Units.inchesToMeters(174.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(7, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19)),
                Rotation2d.fromDegrees(0))
            );
            APRIL_TAG_POS.put(8, new Pose2d(
                new Translation2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19)),
                Rotation2d.fromDegrees(0))
            );
        } 
    }
    
    public static class FieldConstants{
        public static final Pose2d HUB_POSITION = new Pose2d(Units.inchesToMeters(324), Units.inchesToMeters(162),Rotation2d.fromDegrees(-90));
        public static final double FIELD_X_LENGTH = Units.inchesToMeters(648); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(324); // meters
        public static final double HUB_RADIUS = Units.inchesToMeters(26.69); // meters

        public static final double RAMP_X_RIGHT = Units.inchesToMeters(193.25);
        public static final double RAMP_X_LEFT = Units.inchesToMeters(132.375);

        public static Pose2d allianceFlip(Pose2d pose) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        }
        public static Pose2d flip(Pose2d pose) {
            double angle = 180 - pose.getRotation().getDegrees();
            return new Pose2d(
                FIELD_X_LENGTH - pose.getX(),
                pose.getY(),
                Rotation2d.fromDegrees(angle));
          }
    }
}
