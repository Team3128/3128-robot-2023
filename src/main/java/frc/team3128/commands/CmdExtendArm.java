// package frc.team3128.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.team3128.subsystems.Pivot;
// import frc.team3128.subsystems.Telescope;
// import frc.team3128.subsystems.Pivot.PivotAngles;
// import frc.team3128.subsystems.Telescope.TeleDists;

// public class CmdExtendArm extends ParallelCommandGroup {

//     private Pivot pivot;
//     private Telescope telescope;


//     public CmdExtendArm(double anglePos, double teleDist) {
//         pivot = Pivot.getInstance();
//         telescope = Telescope.getInstance();

//         addCommands(
//             if (Math.abs(pivot.getAngle()) > 90) {
//                 new InstantCommand(() -> telescope.startPID(teleDist), telescope);
//             };
//             new InstantCommand(() -> pivot.startPID(anglePos), pivot)
//         );  
//     }
// }

// // TODO: when making button this command runs onTrue and stopping pivot + telescope runs onFalse