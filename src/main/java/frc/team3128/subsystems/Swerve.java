package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.common.swerve.SwerveModule;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

import java.io.FileWriter;

public class Swerve extends SubsystemBase {
    
    private volatile FileWriter txtFile;
    public static double throttle = 0.8;
    private String poseLogger = "";
    private double prevTime = 0; 
    public SwerveDrivePoseEstimator odometry;
    public SwerveModule[] modules;
    public WPI_Pigeon2 gyro;
    private Pose2d estimatedPose;

    private static Swerve instance;
    public boolean fieldRelative;

    private Field2d field;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new WPI_Pigeon2(pigeonID, "drivetrain");
        gyro.configFactoryDefault();
        //zeroGyro();
        fieldRelative = true;
        estimatedPose = new Pose2d();

        // try {
        //     txtFile = new FileWriter(new File(Filesystem.getDeployDirectory(),"pose.txt"));
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };
        resetEncoders();

        odometry = new SwerveDrivePoseEstimator(swerveKinematics, getGyroRotation2d(), getPositions(), 
                                                estimatedPose, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD);


        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        setModuleStates(moduleStates);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addComplex("General","Gyro",gyro,7,2,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("General","Heading",this::getHeading,1,2);
        // // Drivetrain Tab
        NAR_Shuffleboard.addComplex("Field","field",field,0,0,13,7);//.withWidget("Field");
        NAR_Shuffleboard.addData("Drivetrain","Pose",() -> (getPose().toString()),2,0,4,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Gyro",gyro,3,1,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("Drivetrain","Yaw",this::getYaw,4,1);
        NAR_Shuffleboard.addData("Drivetrain","Pitch",this::getPitch,5,1);
        NAR_Shuffleboard.addData("Drivetrain", "Roll", this::getRoll, 0, 2);
        NAR_Shuffleboard.addData("Drivetrain","Heading/Angle",this::getHeading,6,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Drivetrain", this,0,0);
    }

    public Pose2d getPose() {
        return estimatedPose;
    }

    public void addVisionMeasurement(Pose2d pose, double timeStamp) {
        odometry.addVisionMeasurement(new Pose2d(pose.getTranslation(), getGyroRotation2d()), timeStamp);
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void resetOdometry(Pose2d pose) { // TODO: Call this!!!!
        zeroGyro(pose.getRotation().getDegrees());
        //System.out.println(pose.toString());
        odometry.resetPosition(getGyroRotation2d(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
    
    public void toggle() {
        fieldRelative = !fieldRelative;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getPositions());
        estimatedPose = odometry.getEstimatedPosition();
        logPose();
    }

    public void resetAll() {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
        resetEncoders();
    }

    public Rotation2d getRotation2d() {
        return estimatedPose.getRotation();
    }
    
    public double calculateDegreesToTurn(){
        double alpha = getHeading();
        return MathUtil.inputModulus(calculateDesiredAngle() - alpha,-180,180);
    }

    public double calculateDesiredAngle(){
        Pose2d location = getPose().relativeTo(FieldConstants.HUB_POSITION);
        double theta = Math.toDegrees(Math.atan2(location.getY(),location.getX()));
        return MathUtil.inputModulus(theta - 180,-180,180);
    }

    public void logPose() {
        double currTime = Math.floor(Timer.getFPGATimestamp());
        if (prevTime + 1 <= currTime && DriverStation.isEnabled()) {
            poseLogger += estimatedPose.getX() + "," + estimatedPose.getY() + "," + estimatedPose.getRotation().getDegrees() + "," + currTime + "]";
            // try {
            //     txtFile.write(estimatedPose.getX() + "," + estimatedPose.getY() + "," + estimatedPose.getRotation().getDegrees() + "," + currTime + "\n");
            // } catch (Exception e) {
            //     e.printStackTrace();
            // }
            // try {
            //     txtFile.flush();
            // } catch (IOException e) {}
            prevTime = currTime;
            NAR_Shuffleboard.addData("Logger","Positions",poseLogger,0,0);
        }
    }

    public void xlock() {
        setModuleStates(
            new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))}
        );
    }

    public double getYaw() {
        return MathUtil.inputModulus(gyro.getYaw(),-180,180);
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public double getPitch() {
        return gyro.getPitch();
    }
    public double getRoll() {
        return gyro.getRoll();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }
}