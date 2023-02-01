package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdReset;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdGyroBalance;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.CmdTargetPursuit;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private NAR_Camera cam;
    private Pivot pivot;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;
    private NAR_Joystick buttonPad;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger hasTarget;

    public RobotContainer() {
        vision = Vision.getInstance();
        // ConstantsInt.initTempConstants();
        swerve = Swerve.getInstance();
        //pivot = Pivot.getInstance();


        //TODO: Enable all PIDSubsystems so that useOutput runs here

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_Joystick(3);
        CmdMove.setController(controller::getLeftX, controller::getLeftY, controller::getRightX, rightStick::getThrottle);

        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        rightStick.getButton(2).onTrue(new InstantCommand(swerve::toggle));
        rightStick.getButton(3).onTrue(new CmdReset());
        rightStick.getButton(4).onTrue(new InstantCommand(()->swerve.zeroGyro(180)));
        rightStick.getButton(5).onTrue(new RunCommand(()->swerve.drive(new Translation2d(0,0.5),0,true)));
        rightStick.getButton(6).onTrue(new RunCommand(()->swerve.drive(new Translation2d(0.5,0),0,false)));
        rightStick.getButton(7).onTrue(new RunCommand(()->swerve.drive(new Translation2d(0,0.5),0,false)));
        for (int i = 0; i < VisionConstants.LOADING_ZONE.length; i++) {
            leftStick.getButton(i + 1).onTrue(new CmdMove(CmdMove.Type.LOADING, true, VisionConstants.LOADING_ZONE[i])).onFalse(new InstantCommand(()->swerve.stop(),swerve));
        }
        //Test Button to see if skipping works.
        leftStick.getButton(4).onTrue(new CmdMove(CmdMove.Type.NONE,false, new Pose2d(2.3,2.5,Rotation2d.fromDegrees(180)),new Pose2d(4,2.5,Rotation2d.fromDegrees(180)))).onFalse(new InstantCommand(()->swerve.stop()));
        // for (int i = 0; i < VisionConstants.SCORES.length; i++) {
        //     leftStick.getButton(i + 1).onTrue(new CmdMove(CmdMove.Type.SCORE, true, VisionConstants.SCORE_SETUP[i/3],VisionConstants.SCORES[i])).onFalse(new InstantCommand(()->swerve.stop(),swerve));
        // }
        buttonPad.getButton(4).onTrue(new CmdMoveScore(CmdMove.Type.SCORE, true,VisionConstants.SCORE_SETUP,VisionConstants.SCORES_GRID[0])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
        buttonPad.getButton(5).onTrue(new CmdMoveScore(CmdMove.Type.SCORE, true,VisionConstants.SCORE_SETUP,VisionConstants.SCORES_GRID[1])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
        buttonPad.getButton(6).onTrue(new CmdMoveScore(CmdMove.Type.SCORE, true,VisionConstants.SCORE_SETUP,VisionConstants.SCORES_GRID[2])).onFalse(new InstantCommand(()->swerve.stop(),swerve));;
        buttonPad.getButton(1).onTrue(new InstantCommand(()-> CmdMoveScore.SELECTED_GRID = 0));
        buttonPad.getButton(2).onTrue(new InstantCommand(()-> CmdMoveScore.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(new InstantCommand(()-> CmdMoveScore.SELECTED_GRID = 2));


        // rightStick.getButton(3).onTrue(new SequentialCommandGroup(
        //     new CmdDriveUp(),
        //     new CmdGyroBalance()
        // ));

        rightStick.getButton(3).onTrue(new SequentialCommandGroup(
            new CmdDriveUp(),
            new CmdGyroBalance()
        ));
        rightStick.getButton(4).onTrue(new CmdGyroBalance());
        // rightStick.getButton(3).onTrue(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        // rightStick.getButton(4).onTrue(new CmdAlign()).onFalse(new InstantCommand(()-> swerve.stop()));
        // rightStick.getButton(5).onTrue(new InstantCommand(()->swerve.resetOdometry(vision.robotPos(Camera.SHOOTER.hostname))));
        // rightStick.getButton(6).onTrue(new CmdTargetPursuit(Camera.SHOOTER.hostname)).onFalse(new InstantCommand(()->swerve.stop(),swerve));

        // rightStick.getButton(6).whenActive(new InstantCommand(()-> {
        //     if(vision.hasValidTarget(Camera.SHOOTER.hostname)) {
        //         Trajectories.lineCmd(swerve.getPose(),vision.targetPos(Camera.SHOOTER.hostname, swerve.getPose()));
        //     }
        // })).whenInactive(new InstantCommand(swerve::stop,swerve));

        // hasTarget = new Trigger(()-> vision.hasValidTarget(Camera.SHOOTER.hostname))
        // .whenActive(new RunCommand(()-> controller.setRumble(RumbleType.kLeftRumble,1)))
        // .whenInactive(new InstantCommand(()-> controller.setRumble(RumbleType.kLeftRumble, 0)));

        rightStick.getButton(7).onTrue(new InstantCommand(()->pivot.zeroEncoder()));

    }

    public void init() {

    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        swerve.initShuffleboard();
        vision.initShuffleboard();

        NarwhalDashboard.startServer();   
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (NAR_Camera ll : vision.getCameras()) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLED(false);
        }
    }

    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("x", swerve.getPose().getX());
        NarwhalDashboard.put("y", swerve.getPose().getY());
        SmartDashboard.putNumber("LeftX",controller.getLeftX());
        SmartDashboard.putNumber("LeftY",controller.getLeftY());
        SmartDashboard.putNumber("RightX",controller.getRightX());
        SmartDashboard.putNumber("RightY",controller.getRightY());
        NAR_Shuffleboard.update();
        SmartDashboard.putNumber("Pitch",swerve.getPitch());
    }
}
