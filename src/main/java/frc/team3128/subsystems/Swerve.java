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
import frc.team3128.common.swerve.SwerveModule;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Swerve extends SubsystemBase {
    
    public SwerveDrivePoseEstimator odometry;
    public SwerveModule[] modules;
    private FileWriter txtFile;
    private double prevTime;
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
        gyro = new WPI_Pigeon2(pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        fieldRelative = true;
        estimatedPose = new Pose2d();
        prevTime = 0;
        try {
            txtFile = new FileWriter(new File(Filesystem.getDeployDirectory(),"pose.txt"));
        } catch (IOException e) {
        }

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
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        setModuleStates(moduleStates);
    }

    public void initShuffleboard() {
        // General Tab
        NAR_Shuffleboard.addData("General","Gyro",this::getHeading,7,2,2,2).withWidget("Gyro");
        NAR_Shuffleboard.addData("General","Heading",this::getHeading,1,2);
        // Drivetrain Tab
        NAR_Shuffleboard.addComplex("Field","field",field,0,0,13,7).withWidget("Field");
        NAR_Shuffleboard.addData("Drivetrain","Pose",() -> (getPose().toString()),2,0,4,1);
        NAR_Shuffleboard.addComplex("Drivetrain","Gyro",gyro,3,1,2,2).withWidget("Gyro");
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
            module.resetToAbsolute();
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    @Override
    public void periodic() {
        var angle = MathUtil.inputModulus(getHeading(), -180,180);
        odometry.update(Rotation2d.fromDegrees(angle), getPositions());
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
        double time = Timer.getFPGATimestamp();
        if (time >= prevTime + 1) {
            prevTime = time;
            try {
                txtFile.write(position.getX() + "," + position.getY() + "," + estimatedPose.getRotation().getDegrees() + "," + "\n");
            } catch (IOException e) {}
            try {
                txtFile.flush();
            } catch (IOException e) {}
        }
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
    
    public double calculateDegreesToTurn(Pose2d target){
        double alpha = getHeading();
        return MathUtil.inputModulus(calculateDesiredAngle(target) - alpha,-180,180);
    }

    public double calculateDesiredAngle(Pose2d target){
        Pose2d location = getPose().relativeTo(target);
        double theta = Math.toDegrees(Math.atan2(location.getY(),location.getX()));
        return MathUtil.inputModulus(theta - 180,-180,180);
    }
}