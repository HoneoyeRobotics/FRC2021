// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadRecording extends InstantCommand {
  private final DriveTrain m_drivetrain;
  public LoadRecording(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      m_drivetrain.loadAutoDrive();
    } catch (ClassNotFoundException | IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      System.out.println("Recording Loaded");
    }

  }
}
