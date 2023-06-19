package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.common.hardware.camera.Camera;

import frc.team3128.common.swerveNeo.SwerveModuleConstants;

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

    public static class AutoConstants {
        public static final Pose2d PICKUP_1 = new Pose2d(5.9, 0.95, Rotation2d.fromDegrees(0));
        public static final Pose2d PICKUP_2 = new Pose2d(7.1, 2.28, Rotation2d.fromDegrees(45));
        public static final Pose2d PICKUP_3 = new Pose2d(7.1, 3.3, Rotation2d.fromDegrees(-45));
        public static final Pose2d PICKUP_4 = new Pose2d(5.9, 4.55, Rotation2d.fromDegrees(0));

        public static final Pose2d MOBILITY_BOTTOM = new Pose2d(5.5, 0.95, Rotation2d.fromDegrees(0));
        public static final Pose2d MOBILITY_TOP = new Pose2d(5.5, 4.5, Rotation2d.fromDegrees(0));

        public static final Pose2d ClimbSetupInside = new Pose2d(2.2, 2.7, Rotation2d.fromDegrees(0));
        public static final Pose2d ClimbSetupOutsideBot = new Pose2d(5.6, 2.9, Rotation2d.fromDegrees(180));
        public static final Pose2d ClimbSetupOutsideTop = new Pose2d(5.6, 3.3, Rotation2d.fromDegrees(180));

        public static final Pose2d[] STARTING_POINTS = new Pose2d[] {
            new Pose2d(1.85 ,0.5, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,1.05, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,1.65, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,2.15, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,2.75, Rotation2d.fromDegrees(0)),
            new Pose2d(1.85 ,3.3, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,3.85, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,4.45, Rotation2d.fromDegrees(180)),
            new Pose2d(1.85 ,5, Rotation2d.fromDegrees(180))
        };

        public static final double BALANCE_FF = 0.3;
    }

    public static class SwerveConstants {
        public static final int pigeonID = 30; 
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double bumperLength = Units.inchesToMeters(5);
        public static final double trackWidth = Units.inchesToMeters(26); //Hand measure later
        public static final double wheelBase = Units.inchesToMeters(26); //Hand measure later
        public static final double robotLength = bumperLength + trackWidth;
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
        public static final int currentLimit = 40;

        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;


        public static final double TURN_TOLERANCE = 5;

        public static final double DRIVE_TOLERANCE = 0.04;

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
        public static final double rotationKP = 2;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        /* Turning PID Values */
        public static final double turnKP = 0.1;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnKF = 0.1;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.15; // 0.6; // citrus: 0.3 //0.15
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4e-5; //4e-5, //0.05
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.19255;//0.60094; // 0.19225;
        public static final double driveKV = 2.4366;//1.1559;  // 2.4366
        public static final double driveKA = 0.34415; //0.12348; // 0.34415
        public static final double turnTolerance = 2;

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 4.5; //meters per second - 16.3 ft/sec
        public static final double bumpSpeed = 2.5;
        public static final double maxAcceleration = 2.3;
        public static final double maxAngularVelocity = 5; //3; //11.5; // citrus: 10
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration);

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Auto-Balance Constants */
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 0.015; // P (Proportional) constant of a PID loop

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // TODO: Figure out angle offsets
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 20;
            public static final double angleOffset = -162.25;//-157.763671875+180; // -156.357421875;//-46.5 + 90; //104.5;//19.599609375; // 19.51171875;//-51.85546875; // 37.35; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 21;
            public static final double angleOffset = -50.36;//129.375; //126.38671875000001; //23.466 + 90;//-132.25;//311.66015625 - 360; //132.5390625; //311.8359375; //10.45; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 22;
            public static final double angleOffset = -65.74;//-69.697265625+180; //-72.0703125;//-70.751953125; //-70.75; //109.51171875; //38.75; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 23;
            public static final double angleOffset = 124.27;//-54.31640625; //-52.91015625; //-52.9; //306.2109375; //307.6171875; // 58.88; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }


    public static class VisionConstants {

        public static final Camera FRONT = new Camera("Frog", true, 0, 0, 0, 
                                                        new Transform2d(new Translation2d(Units.inchesToMeters(-5.75), 
                                                        Units.inchesToMeters(-11.5)), Rotation2d.fromDegrees(0)));
        public static final Camera BACK = new Camera("Blog", true, 0, 0, 0, 
                                                        new Transform2d(new Translation2d(Units.inchesToMeters(-5.75), 
                                                        Units.inchesToMeters(11.5)), Rotation2d.fromDegrees(180)));

        public static final double SCREEN_WIDTH = 320;
        public static final double SCREEN_HEIGHT = 240;
    
        public static final double HORIZONTAL_FOV = 59.6; //degrees
        public static final double VERTICAL_FOV = 45.7; //degrees

        public static final double TX_THRESHOLD = 3; // degrees

        public static final double POSE_THRESH = 5;

        public static final double AUTO_FF = 0.1;

        public static final double ANGLE_THRESHOLD = 10; // degrees

        public static final double TARGET_AREA = 6.25 * 6.25; //inches

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(1,1,Units.degreesToRadians(10));

        public static final Pose2d[] SCORES = new Pose2d[]{
            new Pose2d(1.90,0.5,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,1.05,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,1.65,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,2.15,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,2.75,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,3.3,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,3.85,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,4.45,Rotation2d.fromDegrees(180)),
            new Pose2d(1.90,4.89,Rotation2d.fromDegrees(180))
        };

        public static final Pose2d[][] SCORES_GRID = new Pose2d[][] {
            new Pose2d[] {SCORES[0], SCORES[3], SCORES[6]},
            new Pose2d[] {SCORES[1], SCORES[4], SCORES[7]},
            new Pose2d[] {SCORES[2], SCORES[5], SCORES[8]}
        };

        public static final boolean[][] RAMP_OVERRIDE = new boolean[][] {
            new boolean[] {true, true, true}, //false, true, true
            new boolean[] {true, true, true}, //false, true, false
            new boolean[] {true, true, true} //true, true, false
        };

        public static final ArrayList<Pose2d> RAMP_AVOID_SCORE = new ArrayList<Pose2d>();

        public static final Pose2d[] SCORE_SETUP = new Pose2d[]{
            new Pose2d(5.3,0.75,Rotation2d.fromDegrees(180)),
            new Pose2d(5.3,2.75,Rotation2d.fromDegrees(180)),
            new Pose2d(5.3,4.6,Rotation2d.fromDegrees(180)),
        };

        public static final Pose2d[] LOADING_ZONE = new Pose2d[] {
            new Pose2d(15.1,6,Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(636.96-76.925),Units.inchesToMeters(265.74+54.5-26), Rotation2d.fromDegrees(90)),
            new Pose2d(15.1,7.3, Rotation2d.fromDegrees(0))
        };

        public static final Pose2d[] RAMP_AVOID_LOADING = new Pose2d[] {
            new Pose2d(10, 0.95, new Rotation2d()),
            new Pose2d(10, 4.5, new Rotation2d())
        };

        public static final Pose2d HPWall_Loading = new Pose2d(12.4, 7.5, new Rotation2d());

        public static final double WALL_PASS = 5.8 + SwerveConstants.robotLength/2.0;

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

            RAMP_AVOID_SCORE.add(new Pose2d(2.1,4.87, Rotation2d.fromDegrees(180)));
            RAMP_AVOID_SCORE.add(new Pose2d(2.1, 0.4, Rotation2d.fromDegrees(180)));
        } 
    }

    public static class PivotConstants {
        public static final double kP = 0.425;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 15;
        public static final double kG = 0.25;
        public static final int PIVOT_MOTOR_ID = 9;
        public static final double ENC_CONV = 360.0/(42.0/16.0*60.0);
        public static final double PIVOT_TOLERANCE = 5.0;
        public static final int PIVOT_CURRENT_LIMIT = 40;
        // public static final int CANCODER_ID = 24;
        public static final int ENC_DIO_ID = 7;
        public static final double ANGLE_OFFSET = 2.5; //get from new encoder
        public static final double GEAR_RATIO = 180.0;
        public static final double PIVOT_HEIGHT = 123; //TBD Above ground (inches)
        public static final double ARM_LENGTH = 56.75; // inches
        
    }

    public static class TelescopeConstants {
        public static final double kP = 2.3;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.1;
        public static final double kG = 0.5;
        public static final int TELE_MOTOR_ID = 10;
        public static final double TELE_MOTOR_POWER = 0.5;
        public static final double ENC_CONV = (1/5.0) * 2 * Math.PI * 0.4; //55.0 /35.0
        public static final double MIN_DIST = 11.5;
        public static final double MAX_DIST = 40;
        public static final double TELE_OFFSET = 0;
        public static final double TELE_TOLERANCE = 1;
        public static final int TELE_CURRENT_LIMIT = 40;
        public static final boolean isReversed = true;

        public static final double ARM_LENGTH = 40; // inches
        public static final int SOLENOID_FORWARD_CHANNEL_ID = 5; 
        public static final int SOLENOID_BACKWARD_CHANNEL_ID = 2; 
    }

    public static class ArmConstants {

        public enum ArmPosition {
            TOP_CONE(112, 38.5, 0, true), // 112,44 
            TOP_CUBE(105, 38.5, 0, false), //98,35
            MID_CONE(105, 20.5, 1, true), //98,22.5
            MID_CUBE(90, 22, 1, false), //87,15
            LOW_FLOOR(45, 11.5, 2, false), 

            NEUTRAL(5, 11.5, null, null), //pivot should be 0

            HP_SHELF_CONE(110, 17.5, null, null), //105
            HP_SHELF_CUBE(108, 17.5, null, null), //105
            GROUND_PICKUP(37, 26.5, null, null),
            GROUND_PICKUP_CONE(37, 26.5, null, null), 
            GROUND_PICKUP_CUBE(37.5, 26, null, null), 
            CONE_POLE(-40, 11.5, null, null),
            AVOID_INTAKE(90, 11.5, null, null);
    
            
            public final double pivotAngle;
            public final double teleDist;
            public final Integer height;
            public final Boolean cone;
    
            private ArmPosition(double pivotAngle, double teleDist, Integer height, Boolean cone) {
                this.pivotAngle = pivotAngle;
                this.teleDist = teleDist;
                this.height = height;
                this.cone = cone;
            }
        }
    
    }
    
    public static class FieldConstants{
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

        public static final double cableBumpInnerX = Units.inchesToMeters(149.5);
        public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
        public static final Translation2d[] cableBumpCorners =
        new Translation2d[] {
          new Translation2d(cableBumpInnerX, 0.0),
          new Translation2d(cableBumpInnerX, chargingStationRightY),
          new Translation2d(cableBumpOuterX, 0.0),
          new Translation2d(cableBumpOuterX, chargingStationRightY)
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

    public static class IntakeConstants {

        public static final double INTAKE_DEPLOYED_POSITION_BOUNDRY = 0;

        public static final double CURRENT_THRESHOLD = 5;
        public static final double ABSOLUTE_THRESHOLD = 30;

        public static final double ROLLER_POWER = 0.75;
        public static final double OUTTAKE_POWER = 0.3;

        public static final double STALL_POWER = 0.1;

        public static final double kP = 0.065;
        public static final double kI = 0;
        public static final double kD = 0; // 0.001

        public static final double kF = 0.2;

        public static final double ROTATOR_GEAR_RATIO = 1.0 / 30.0;

        public static final double ENCODER_CONVERSION_FACTOR_TO_DEGREES = 360;
        public static final double ANGLE_OFFSET = 138+35; 

        public static final int ENCODER_DIO_ID = 8;

        public static final double VELOCITY_SETPOINT = 0.5;
        public static final double INTAKE_TOLERANCE = 7.5;

        //Motor IDs
        public static final int INTAKE_PIVOT_ID = 12;
        public static final int INTAKE_ROLLERS_ID = 11;

        //Sensor IDs
        public static final int INTAKE_SENSOR_ID = 1;
        // public static final int CONE_SENSOR_ID = 0;
        // public static final int INTAKE_SENSOR_LEFT_ID = 1;
        // public static final int INTAKE_SENSOR_RIGHT_ID = 2;
        
    }

    public static class ManipulatorConstants{
        public static final int ROLLER_MOTOR_ID = 13;
        public static final double ROLLER_POWER = 0.6;
        public static final double STALL_POWER = 0.25;

        public static final double CURRENT_THRESHOLD = 5;
        public static final double ABSOLUTE_THRESHOLD = 20;

        public static final double ROLLER_VOLTAGE = 8;
    }

    public static class BalanceConstants{
        public static final double turnKP = 0.05;
        public static final double turnKI = 0;
        public static final double turnKD = .005;
        public static final double TURN_TOLERANCE = 1.5;
        public static final double CHARGE_STATION_X = 5;
        public static final double CHARGE_STATION_Y = 5;
    }

    public static class LedConstants{
        public static class Green{
            public static final int HUE = 60;
            public static final int SATURATION = 255;
            public static final int VALUE = 255;
        }

        public static class Blue{
            public static final int HUE = 120;
            public static final int SATURATION = 255;
            public static final int VALUE = 255;
        }

        public static class Red{
            public static final int HUE = 0;
            public static final int SATURATION = 255;
            public static final int VALUE = 255;
        }

        public static class Yellow{
            public static final int HUE = 30;
            public static final int SATURATION = 255;
            public static final int VALUE = 255;
        }

        public static class Purple{
            public static final int HUE = 130;
            public static final int SATURATION = 255;
            public static final int VALUE = 50;
        }

        public static class Off{
            public static final int HUE = 0;
            public static final int SATURATION = 0;
            public static final int VALUE = 0;
        }

        public static final int PORT = 0; 
        public static final int LENGTH = 288;
    }


}


