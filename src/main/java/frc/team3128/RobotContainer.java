package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.CmdSystemCheck;
import frc.team3128.commands.CmdSystemCheckFancy;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMoveArm;

import static frc.team3128.Constants.FieldConstants.*;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.commands.CmdBalance;
import frc.team3128.commands.CmdBangBangBalance;
import static frc.team3128.commands.CmdManager.*;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.TelescopeConstants;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_ButtonBoard;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Telescope;
import frc.team3128.subsystems.Vision;
import static frc.team3128.Constants.ArmConstants.*;

import java.util.function.BooleanSupplier;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private Intake intake;
    private Pivot pivot;
    private Telescope telescope;
    private Manipulator manipulator;
    private Led led;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;
    private NAR_XboxController operatorController;

    public static NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    public static BooleanSupplier DEBUG = ()-> false; 

    private Trigger inProtected;
    private Trigger isAuto;

    public RobotContainer() {
        NAR_Shuffleboard.addData("DEBUG", "DEBUG", ()-> DEBUG.getAsBoolean(), 0, 1);
        var x = NAR_Shuffleboard.addData("DEBUG", "TOGGLE", false, 0, 0).withWidget("Toggle Button");
        DEBUG = ()-> x.getEntry().getBoolean(false);

        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        vision = Vision.getInstance();
        pivot = Pivot.getInstance();
        telescope = Telescope.getInstance();
        manipulator = Manipulator.getInstance();
        led = Led.getInstance();

        isAuto = new Trigger(() -> Vision.AUTO_ENABLED);

        //TODO: Enable all PIDSubsystems so that useOutput runs here
        // pivot.enable();
        // telescope.enable();

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        operatorController = new NAR_XboxController(4);

        CmdMove.setController(controller::getLeftX, controller::getLeftY, controller::getRightX, ()-> Swerve.throttle);

        // commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, true));
        
        
        //uncomment line below to enable driving
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton("A").onTrue(new InstantCommand(()-> Vision.AUTO_ENABLED = !Vision.AUTO_ENABLED));
        controller.getButton("RightTrigger").onTrue(new InstantCommand(()-> Swerve.throttle = 1)).onFalse(new InstantCommand(()-> Swerve.throttle = 0.8));
        controller.getButton("LeftTrigger").onTrue(new InstantCommand(()-> Swerve.throttle = .25)).onFalse(new InstantCommand(()-> Swerve.throttle = 0.8));
        controller.getButton("X").onTrue(new RunCommand(()-> swerve.xlock(), swerve)).onFalse(new InstantCommand(()-> swerve.stop(),swerve));
        controller.getButton("B").onTrue(new InstantCommand(()-> swerve.resetEncoders()));
        controller.getButton("Y").onTrue(Commands.sequence(CmdExtendRelease(), new InstantCommand(()-> Swerve.throttle = 0.25)))
                                        .onFalse(Commands.sequence(CmdManipOuttake(),
                                            new WaitCommand(0.35), CmdStopManip(),
                                            new InstantCommand(()-> Swerve.throttle = 0.8), 
                                            new CmdMoveArm(ArmPosition.NEUTRAL)));

        controller.getButton("RightBumper").onTrue(CmdIntakeOuttake()).onFalse(CmdStopIntake());
        controller.getButton("LeftBumper").onTrue(CmdIntake()).onFalse(Commands.sequence(
            new InstantCommand(() -> intake.set(Intake.objectPresent ? IntakeConstants.STALL_POWER : 0), intake),
            CmdExtendIntake(Intake.IntakeState.RETRACTED),
            new WaitUntilCommand(()-> intake.atSetpoint())));

        // rightStick.getButton(4).onTrue(new StartEndCommand(() ->telescope.retract(), () -> {telescope.stopTele(); telescope.zeroEncoder(TelescopeConstants.TELE_OFFSET);}).until(() -> !telescope.getLimitSwitch()));
        
        rightStick.getButton(1).onTrue(new InstantCommand(()->swerve.zeroGyro()));
        //rightStick.getButton(2).onTrue(new InstantCommand(()->vision.visionReset()));
        
        rightStick.getButton(2).onTrue(new InstantCommand(()->telescope.releaseBrake()));
        rightStick.getButton(3).onTrue(new InstantCommand(()->telescope.zeroEncoder()));
        rightStick.getButton(4).onTrue(new InstantCommand(()->pivot.resetPivot()));
        
        rightStick.getButton(5).onTrue(CmdPivot(0));
        rightStick.getButton(6).onTrue(CmdTele(11.5));

        //rightStick.getButton(7).onTrue(new CmdBalance());
        rightStick.getButton(7).onTrue(Commands.sequence(
                                            Commands.deadline(Commands.sequence(new WaitUntilCommand(()-> Math.abs(swerve.getPitch()) > 6), new CmdBangBangBalance()), new CmdBalance()), 
                                            //new RunCommand(()-> swerve.drive(new Translation2d(CmdBalance.DIRECTION ? -0.25 : 0.25,0),0,true)).withTimeout(0.5), 
                                            new RunCommand(()->Swerve.getInstance().xlock(), Swerve.getInstance())));

        rightStick.getButton(8).onTrue(CmdStopIntake());
    
        // manual controls
        rightStick.getButton(9).onTrue(new InstantCommand(()->telescope.retract(), telescope)).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        rightStick.getButton(10).onTrue(new InstantCommand(()->telescope.extend(), telescope)).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        
        rightStick.getButton(11).onTrue(new InstantCommand(()->pivot.setPower(0.4), pivot)).onFalse(new InstantCommand(()->pivot.setPower(0.0), pivot));
        rightStick.getButton(12).onTrue(new InstantCommand(()->pivot.setPower(-0.4), pivot)).onFalse(new InstantCommand(()->pivot.setPower(0.0), pivot));

        rightStick.getButton(13).onTrue(CmdManipGrab(true));
        rightStick.getButton(14).onTrue(CmdManipGrab(false));

        rightStick.getButton(15).onTrue(CmdStopManip());
        rightStick.getButton(16).onTrue(CmdManipOuttake());

        // rightStick.getUpPOVButton().onTrue(new CmdSystemCheck());
        // rightStick.getDownPOVButton().onTrue(new InstantCommand(()-> CmdSystemCheck.systemCheck++));

        rightStick.getUpPOVButton().onTrue(new CmdSystemCheckFancy());
        rightStick.getDownPOVButton().onTrue(new InstantCommand(()-> CmdSystemCheckFancy.repeat = true));
        rightStick.getRightPOVButton().onTrue(new InstantCommand(()->{CmdSystemCheckFancy.systemCheck++;}));
        rightStick.getLeftPOVButton().onTrue(new InstantCommand(()->{CmdSystemCheckFancy.systemCheck--;}));

        buttonPad.getButton(13).onTrue(new CmdMoveArm(ArmPosition.NEUTRAL).andThen(new InstantCommand(()->manipulator.stallPower(), manipulator)));

        buttonPad.getButton(14).onTrue(Commands.sequence(
            new InstantCommand(()-> pivot.startPID(282)),
            CmdManipGrab(true)));
            //pivot.setPower(0); telescope.stopTele(); manipulator.stopRoller(); swerve.stop(); intake.stop();}, pivot, telescope, swerve, manipulator, intake));
        buttonPad.getButton(16).onTrue(
            //CmdShelfPickup(true)
            Commands.sequence(
            new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(()-> pivot.startPID(283.5)),
            CmdManipGrab(true),
            new WaitCommand(0.5),
            new CmdMoveArm(ArmPosition.NEUTRAL)
            ));
        buttonPad.getButton(15).onTrue(
            //CmdShelfPickup(false)
            Commands.sequence(
                new InstantCommand(()-> Vision.AUTO_ENABLED = false),
            new WaitUntilCommand(()-> Vision.AUTO_ENABLED),
            new InstantCommand(()-> pivot.startPID(288)),
            CmdManipGrab(false),
            new WaitCommand(0.5),
            new CmdMoveArm(ArmPosition.NEUTRAL)
            ));

        //rightStick.getUpPOVButton().onTrue(new InstantCommand(()-> led.setAllianceColor()));
        //rightStick.getDownPOVButton().onTrue(new InstantCommand(()-> led.setAutoColor()));

        //Intake Buttons
        // leftStick.getButton(9).onTrue(new InstantCommand(()-> intake.intake(), intake)).onFalse(new InstantCommand(()-> intake.stopRollers(), intake));
        // leftStick.getButton(10).onTrue(new InstantCommand(()-> intake.outtake(), intake)).onFalse(new InstantCommand(()-> intake.stopRollers(), intake));
        // leftStick.getButton(11).onTrue(new InstantCommand(()-> intake.startPID(90), intake));
        // leftStick.getButton(1).onTrue(new InstantCommand(()->intake.moveIntake(0.2), intake)).onFalse(new InstantCommand(()->intake.stop(), intake));
        // leftStick.getButton(2).onTrue(new InstantCommand(()->intake.moveIntake(-0.2), intake)).onFalse(new InstantCommand(()->intake.stop(), intake));
        // leftStick.getButton(3).onTrue(CmdIntake());
        
        buttonPad.getButton(5).onTrue(
            CmdScore(false, ArmPosition.LOW_FLOOR, 1)
        );
        buttonPad.getButton(8).onTrue(
            CmdScore(false, ArmPosition.MID_CUBE, 1)
        );
        buttonPad.getButton(11).onTrue(
            CmdScore(false, ArmPosition.TOP_CUBE, 1)
        );
        buttonPad.getButton(1).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 0 : 2;
        }));
        buttonPad.getButton(2).onTrue(new InstantCommand(()-> Vision.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(new InstantCommand(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 2 : 0;
        }));

        operatorController.getButton("Back").onTrue(new InstantCommand(()-> Vision.MANUAL = !Vision.MANUAL));
        operatorController.getButton("Y").onTrue(new InstantCommand(()-> telescope.zeroEncoder()));
        operatorController.getButton("X").onTrue(CmdStopManip());
        operatorController.getButton("A").onTrue(new CmdMoveArm(ArmPosition.NEUTRAL).alongWith(CmdExtendIntake(Intake.IntakeState.RETRACTED)));

        operatorController.getButton("LeftBumper").onTrue(CmdManipGrab(false)).onFalse(CmdStopManip());
        operatorController.getButton("RightBumper").onTrue(CmdManipGrab(true)).onFalse(CmdStopManip());

        operatorController.getButton("LeftTrigger").onTrue(CmdManipOuttake());
        operatorController.getButton("RightTrigger").onTrue(CmdManipOuttake());

        operatorController.getUpPOVButton().onTrue(new InstantCommand(()-> intake.moveIntake(0.2), intake)).onFalse(new InstantCommand(()-> intake.stop(), intake));
        operatorController.getDownPOVButton().onTrue(new InstantCommand(()-> intake.moveIntake(-0.2), intake)).onFalse(new InstantCommand(()-> intake.stop(), intake));
        operatorController.getRightPOVButton().onTrue(CmdIntakeIntake()).onFalse(new InstantCommand(()-> intake.set(intake.hasObjectPresent() ? 0.1 : 0), intake));
        operatorController.getLeftPOVButton().onTrue(CmdIntakeOuttake()).onFalse(CmdStopIntake());

        
        // on false pidlock to getmeasurement
        operatorController.getButton("LeftPosY").onTrue(new InstantCommand(()->pivot.setPower(0.2), pivot)).onFalse(new InstantCommand(()->pivot.setPower(0.0), pivot));
        operatorController.getButton("LeftNegY").onTrue(new InstantCommand(()->pivot.setPower(-0.2), pivot)).onFalse(new InstantCommand(()->pivot.setPower(0.0), pivot));
        operatorController.getButton("RightNegY").onTrue(new InstantCommand(()->telescope.retract(), telescope)).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));
        operatorController.getButton("RightPosY").onTrue(new InstantCommand(()->telescope.extend(), telescope)).onFalse(new InstantCommand(() -> telescope.stopTele(), telescope));

        // isAuto.onTrue(new InstantCommand(() -> led.setAutoColor())).onFalse(new InstantCommand(()-> led.setAllianceColor()));

        inProtected = new Trigger(
            () -> {
                Pose2d pose = Swerve.getInstance().getPose();
                if (DriverStation.getAlliance() == Alliance.Red) {
                    return ((pose.getY() < midY + robotLength/2 && pose.getX() < outerX + robotLength/2) || 
                        (pose.getY() < leftY + robotLength/2 && pose.getX() < midX + robotLength/2)) ||
                        ((pose.getY() > 6.85 && pose.getX() > FIELD_X_LENGTH - 6.70) || (pose.getY() > 5.50 && pose.getX() > FIELD_X_LENGTH - 3.30));
                }
                return ((pose.getY() < midY + robotLength/2 && pose.getX() > FIELD_X_LENGTH - outerX - robotLength/2) || 
                    (pose.getY() < leftY + robotLength/2 && pose.getX() > FIELD_X_LENGTH - midX - robotLength/2)) ||
                    ((pose.getY() > 6.85 && pose.getX() < 6.70) || (pose.getY() > 5.50 && pose.getX() < 3.30));
            }
        );
        inProtected.onTrue(new InstantCommand(()-> controller.startVibrate())).onFalse(new InstantCommand(()-> controller.stopVibrate()));
    }

    public void init() {
        Vision.AUTO_ENABLED = false;
        if (DriverStation.getAlliance() == Alliance.Red) {
            buttonPad.getButton(4).onTrue(
                CmdScore(false, ArmPosition.LOW_FLOOR, 0)
            );
            buttonPad.getButton(6).onTrue(
                CmdScore(false, ArmPosition.LOW_FLOOR, 2)
            );
            buttonPad.getButton(7).onTrue(
                CmdScore(false, ArmPosition.MID_CONE, 0)
            );
            buttonPad.getButton(9).onTrue(
                CmdScore(false, ArmPosition.MID_CONE, 2)
            );
            buttonPad.getButton(10).onTrue(
                CmdScore(false, ArmPosition.TOP_CONE, 0)
            );
            buttonPad.getButton(12).onTrue(
                CmdScore(false, ArmPosition.TOP_CONE, 2)
            );
            
        }
        else {
            buttonPad.getButton(6).onTrue(
                CmdScore(false, ArmPosition.LOW_FLOOR, 0)
            );
            buttonPad.getButton(4).onTrue(
                CmdScore(false, ArmPosition.LOW_FLOOR, 2)
            );
            buttonPad.getButton(9).onTrue(
                CmdScore(false, ArmPosition.MID_CONE, 0)
            );
            buttonPad.getButton(7).onTrue(
                CmdScore(false, ArmPosition.MID_CONE, 2)
            );
            buttonPad.getButton(12).onTrue(
                CmdScore(false, ArmPosition.TOP_CONE, 0)
            );
            buttonPad.getButton(10).onTrue(
                CmdScore(false, ArmPosition.TOP_CONE, 2)
            );
        }
    }

    private void initDashboard() {
        if (DEBUG.getAsBoolean()) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        swerve.initShuffleboard();
        vision.initShuffleboard();
        intake.initShuffleboard();
        telescope.initShuffleboard();
        pivot.initShuffleboard();
        manipulator.initShuffleboard();
        CmdSystemCheckFancy.initShuffleboard();

        NarwhalDashboard.startServer();
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        // for (NAR_Camera cam : vision.getCameras()) {
        //     NarwhalDashboard.addLimelight(cam);
        // }
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
