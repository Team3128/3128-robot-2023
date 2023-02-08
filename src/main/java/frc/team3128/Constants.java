package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.swerve.FalconConversions;
import frc.team3128.common.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

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
        public static final double trackWidth = Units.inchesToMeters(26); 
        public static final double wheelBase = Units.inchesToMeters(26); 
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; 
        public static final double angleGearRatio = (150.0 / 7.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

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

        public static final double DRIVE_TOLERANCE = 0.025;

        /* Translation PID Values */
        public static final double translationKP = 3;
        public static final double translationKI = 0;
        public static final double translationKD = 0;

        /* Translation PID Values */
        public static final double distanceKP = 3;
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
        public static final double driveKP = 0.05; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // TODO: sysid this
        public static final double driveKS = 0.60094;
        public static final double driveKV = 1.1559; 
        public static final double driveKA = 0.12348; 

        public static final double turnTolerance = 2;

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 4; //meters per second - 16.3 ft/sec
        public static final double maxAcceleration = 2;
        public static final double maxAngularVelocity = 2; //3; //11.5; // citrus: 10
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Auto-Balance Constants */
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 0.015; // P (Proportional) constant of a PID loop

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 20;
            public static final double angleOffset = -157.763671875; // deg
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 21;
            public static final double angleOffset = 129.375; // deg
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 22;
            public static final double angleOffset = -69.697265625; // deg
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 23;
            public static final double angleOffset = -54.31640625; // deg
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static class VisionConstants {

        public static final Camera FRONT = new Camera("Frog", true, 0, 0, 0, 
                                                        new Transform2d(new Translation2d(Units.inchesToMeters(-7), 
                                                        Units.inchesToMeters(-13.75)), Rotation2d.fromDegrees(0)));

        public static final double SCREEN_WIDTH = 320;
        public static final double SCREEN_HEIGHT = 240;
    
        public static final double HORIZONTAL_FOV = 59.6; //degrees
        public static final double VERTICAL_FOV = 45.7; //degrees

        public static final double TX_THRESHOLD = 3; // degrees

        public static final double ANGLE_THRESHOLD = 10; // degrees

        public static final double TARGET_AREA = 6.25 * 6.25; //inches

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(1,1,Units.degreesToRadians(10));

        public static final Pose2d[] SCORES = new Pose2d[]{
            new Pose2d(1.95,0.5,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,1.05,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,1.65,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,2.15,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,2.75,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,3.3,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,3.85,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,4.45,Rotation2d.fromDegrees(180)),
            new Pose2d(1.95,5,Rotation2d.fromDegrees(180))
        };

        public static final Pose2d[][] SCORES_GRID = new Pose2d[][] {
            new Pose2d[] {SCORES[0], SCORES[3], SCORES[6]},
            new Pose2d[] {SCORES[1], SCORES[4], SCORES[7]},
            new Pose2d[] {SCORES[2], SCORES[5], SCORES[8]}
        };

        public static final boolean[][] RAMP_OVERRIDE = new boolean[][] {
            new boolean[] {false, true, true},
            new boolean[] {false, true, false},
            new boolean[] {true, true, false}
        };

        public static final ArrayList<Pose2d> RAMP_AVOID_SCORE = new ArrayList<Pose2d>();

        public static final Pose2d[] SCORE_SETUP = new Pose2d[]{
            new Pose2d(5.3,0.75,Rotation2d.fromDegrees(180)),
            new Pose2d(5.3,2.75,Rotation2d.fromDegrees(180)),
            new Pose2d(5.3,4.6,Rotation2d.fromDegrees(180)),
        };

        public static final Pose2d[] LOADING_ZONE = new Pose2d[] {
            new Pose2d(15.5,7.5,Rotation2d.fromDegrees(0)),
            new Pose2d(15.5,5.8, Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(636.96-76.925),Units.inchesToMeters(265.74+54.5-26), Rotation2d.fromDegrees(90))
        };

        public static final Pose2d RAMP_AVOID_LOADING = new Pose2d(10, 1.12, new Rotation2d());

        public static final HashMap<Integer,Pose2d> APRIL_TAG_POS = new HashMap<Integer,Pose2d>();

        public static final HashMap<Integer,Pose2d> TestTags = new HashMap<Integer, Pose2d>();

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

            TestTags.put(8, APRIL_TAG_POS.get(3));
            TestTags.put(7, APRIL_TAG_POS.get(2));
            TestTags.put(6,APRIL_TAG_POS.get(1));

            RAMP_AVOID_SCORE.add(new Pose2d(1.7,4.35, Rotation2d.fromDegrees(180)));
            RAMP_AVOID_SCORE.add(new Pose2d(1.7, 1.12, Rotation2d.fromDegrees(180)));
        } 
    }

    public static class PivotConstants {
        public static final double kP = 5E-1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 1;
        public static final int PIVOT_MOTOR_ID = 9;
        public static final double ENC_CONV = 360.0/(42.0/16.0*60.0);
        public static final double PIVOT_TOLERANCE = 3.0;
        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 90;
        public static final int PIVOT_CURRENT_LIMIT = 40;
        
        public static final double PIVOT_HEIGHT = 123; //TBD Above ground (inches)
        public static final double ARM_LENGTH = 56.75; // inches
        
    }

    public static class TelescopeConstants {
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.1;
        public static final double kG = 0.5;
        public static final int TELE_MOTOR_ID = 10;
        public static final double TELE_MOTOR_POWER = 0.5;
        public static final double ENC_CONV = (1/5.0) * 2 * Math.PI * 0.4;
        public static final double MIN_DIST = 11.5;
        public static final double MAX_DIST = 48;
        public static final double TELE_TOLERANCE = 0.5;
        public static final int TELE_CURRENT_LIMIT = 40;

        public static final double ARM_LENGTH = 56.75; // inches
    }

    public static class ArmConstants {
        public enum ScoringPosition {
            TOP_CONE(180 - 81.666, 56.75), 
            TOP_CUBE(180 - 92.221, 56.75), 
            MID_CONE(180 - 95.559, 40.027), 
            MID_CUBE(180 - 110.041, 39.031), 
            LOW_FLOOR(180 - 155.114, 16.0), 
            HP_PICK_UP(0.0, 16.0), 
            INT_PICK_UP(0.0, 16.0), 
            NEUTRAL(0.0, 11.5);
    
            public final double pivotAngle;
            public final double teleDist;
    
            private ScoringPosition(double pivotAngle, double teleDist) {
                this.pivotAngle = pivotAngle;
                this.teleDist = teleDist;
            }
        }
    }
    
    public static class FieldConstants{
        public static final Pose2d HUB_POSITION = new Pose2d(Units.inchesToMeters(324), Units.inchesToMeters(162),Rotation2d.fromDegrees(-90));
        public static final double FIELD_X_LENGTH = Units.inchesToMeters(651.25); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(315.5); // meters
        public static final double HUB_RADIUS = Units.inchesToMeters(26.69); // meters

        public static final double LOADING_X_LEFT = 13.2; // meters
        public static final double LOADING_X_RIGHT = FIELD_X_LENGTH;
        public static final double tapeWidth = Units.inchesToMeters(2.0);
        public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
        public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;

        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners =
        new Translation2d[] {
          new Translation2d(chargingStationInnerX, chargingStationRightY),
          new Translation2d(chargingStationInnerX, chargingStationLeftY),
          new Translation2d(chargingStationOuterX, chargingStationRightY),
          new Translation2d(chargingStationOuterX, chargingStationLeftY)
        };


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

    public static class LedConstants{
        public static class Yellow{
            public static final int HUE = 0;
            public static final int SATURATION = 0;
            public static final int VALUE = 0;
        }
        public static class Purple{
            public static final int HUE = 1;
            public static final int SATURATION = 1;
            public static final int VALUE = 1;
        }

        public static final int PORT = 0;
        public static final int LENGTH = 60;
    }

    public static class ManipulatorConstants{
        public static final int SOLENOID_FORWARD_CHANNEL_ID = 4;
        public static final int SOLENOID_BACKWARD_CHANNEL_ID = 3;
    }

    public static class HopperConstants {
        public static final int SERIALIZER_ID = 2;
        public static final double SERIALIZER_POWER = 0;

        public static final int SENSOR_LEFT_ID = 0;
        public static final int SENSOR_RIGHT_ID = 1;
    }

    public static class BalanceConstants{
        public static final double turnKP = 0.05;
        public static final double turnKI = 0;
        public static final double turnKD = .005;
        public static final int TURN_TOLERANCE = 1;
        public static final double CHARGE_STATION_X = 5;
        public static final double CHARGE_STATION_Y = 5;
    }


}
