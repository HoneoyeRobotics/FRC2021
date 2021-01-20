// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.controller.*;
// import edu.wpi.first.wpilibj.geometry.*;
// import edu.wpi.first.wpilibj.trajectory.*;
// import edu.wpi.first.wpilibj.trajectory.constraint.*;
// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.commands.*;
// import frc.robot.Constants;

// public class AutoManualTest extends SequentialCommandGroup {
//   /**
//    * Creates a new AutoPathFinderTest.
//    */
//   public AutoManualTest(DriveTrain drivetrain) {
//         // Create a voltage constraint to ensure we don't accelerate too fast
//         var autoVoltageConstraint =
//         new DifferentialDriveVoltageConstraint(
//             new SimpleMotorFeedforward(Constants.Analysis_ks,
//                                        Constants.Analysis_kv,
//                                        Constants.Analysis_ka),
//             Constants.kDriveKinematics,
//             10);

//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(Constants.kMaxSpeed,
//                              Constants.kMaxAcceleration)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(Constants.kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);

//     // An example trajectory to follow.  All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(
//             new Translation2d(1, 1),
//             new Translation2d(2, -1)
//         ),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         // Pass config
//         config
//     );

//     RamseteCommand ramseteCommand = new RamseteCommand(
//         exampleTrajectory,
//         drivetrain::getPose,
//         new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
//         new SimpleMotorFeedforward(Constants.Analysis_ks,
//                                    Constants.Analysis_kv,
//                                    Constants.Analysis_ka),
//         Constants.kDriveKinematics,
//         drivetrain::getWheelSpeeds,
//         new PIDController(Constants.ControllerGain_kP, 0, 0),
//         new PIDController(Constants.ControllerGain_kP, 0, 0),
//         // RamseteCommand passes volts to the callback
//         drivetrain::tankDriveVolts,
//         drivetrain
//     );
         
//       addCommands(ramseteCommand, new StopDrives(drivetrain));
    
//   }
// }