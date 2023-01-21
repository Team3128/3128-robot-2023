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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.common.swerve.SecondOrderChassisSpeeds;
import frc.team3128.common.swerve.SecondOrderSwerveDriveKinematics;
import frc.team3128.common.swerve.SwerveModule;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.common.swerve.SecondOrderSwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Swerve extends SubsystemBase {
    
    public SwerveDrivePoseEstimator odometry;
    public SwerveModule[] modules;
    public WPI_Pigeon2 gyro;
    private Pose2d estimatedPose;

    private static Swerve instance;
    public boolean fieldRelative;

    private Field2d field;

    //Variables to help calculate accelerations (For second-order)
    private Translation2d previousTranslation;
    private double previousTime;
    private double previousRotation;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new WPI_Pigeon2(pigeonID, "drivetrain");
        gyro.configFactoryDefault();
        zeroGyro();
        fieldRelative = true;
        estimatedPose = new Pose2d();

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };
        resetEncoders();

        odometry = new SwerveDrivePoseEstimator(
            swerveKinematics, 
            new Rotation2d(), 
            getPositions(), 
            estimatedPose, 
            SVR_STATE_STD, 
            SVR_VISION_MEASUREMENT_STD);


        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

        if(previousTranslation == null) {
            previousTranslation = translation;
            previousTime = Timer.getFPGATimestamp();
            previousRotation = rotation;
        }

        double dt = (Timer.getFPGATimestamp() - previousTime);

        //Get SecondOrderChassisSpeeds
        SecondOrderChassisSpeeds secondOrderChassisSpeeds = new SecondOrderChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            (translation.getX() - previousTranslation.getX()) / dt,
            (translation.getY() - previousTranslation.getY()) / dt,
            rotation-previousRotation/dt //TODO: I'm not sure if rotation is an angular velocity
        );
        
        SecondOrderSwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? SecondOrderChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, secondOrderChassisSpeeds.axMetersPerSecondSq,
                secondOrderChassisSpeeds.ayMetersPerSecondSq, secondOrderChassisSpeeds.alphaRadiansPerSecondSq, getRotation2d())
                : secondOrderChassisSpeeds);
        //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        setModuleStates(moduleStates);
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addComplex("General","Gyro",gyro,7,2,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("General","Heading",this::getHeading,1,2);
        // Drivetrain Tab
        NAR_Shuffleboard.addComplex("Field","field",field,0,0,13,7);//.withWidget("Field");
        NAR_Shuffleboard.addData("Drivetrain","Pose",() -> (getPose().toString()),2,0,4,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Gyro",gyro,3,1,2,2);//.withWidget("Gyro");
        NAR_Shuffleboard.addData("Drivetrain","Yaw",this::getYaw,4,1);
        NAR_Shuffleboard.addData("Drivetrain","Pitch",this::getPitch,5,1);
        NAR_Shuffleboard.addData("Drivetrain","Heading/Angle",this::getHeading,6,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Drivetrain", this,0,0);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public Pose2d getPose() {
        return estimatedPose;
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    public void resetOdometry(Pose2d pose) { // TODO: Call this!!!!
        zeroGyro(pose.getRotation().getDegrees());
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

    public void setModuleStates(SecondOrderSwerveModuleState[] desiredStates) {
        SecondOrderSwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxWheelSpeed, maxAcceleration);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getPositions());
        for(SwerveModule module : modules){
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);    
        }
        estimatedPose = odometry.getEstimatedPosition();
        Translation2d position = estimatedPose.getTranslation();
        SmartDashboard.putNumber("Robot X", position.getX());
        SmartDashboard.putNumber("Robot Y", position.getY());
        SmartDashboard.putNumber("Robot Gyro", getGyroRotation2d().getDegrees());
        SmartDashboard.putString("POSE2D",getPose().toString());
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Rotation2d getRotation2d() {
        return estimatedPose.getRotation();
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
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
}