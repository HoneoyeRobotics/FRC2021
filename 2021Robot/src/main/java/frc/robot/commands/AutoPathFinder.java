/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.*;
import frc.robot.Constants;

public class AutoPathFinder extends SequentialCommandGroup {
  /**
   * Creates a new AutoPathFinderTest.
   */
  public AutoPathFinder(DriveTrain drivetrain, String pathName) {
    // Use addRequirements() here to declare subsystem dependencies.
    RamseteCommand ramseteCommand = null;
    String trajectoryJSON = "paths/" + pathName + ".wpilib.json";

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);

    Trajectory trajectory = null;

    try {

      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.Analysis_ks, Constants.Analysis_kv, Constants.Analysis_ka),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.ControllerGain_kP, 0, Constants.ControllerGain_kD),
          new PIDController(Constants.ControllerGain_kP, 0, Constants.ControllerGain_kD),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);

     
      addCommands(ramseteCommand, new StopDrives(drivetrain));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      addCommands(new StopDrives(drivetrain));
    }
    
  }

}
