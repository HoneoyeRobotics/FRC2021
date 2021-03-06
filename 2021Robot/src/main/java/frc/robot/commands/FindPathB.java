// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.PowercellSystem;

public class FindPathB extends InstantCommand {
  /** Creates a new FindPathA. */
  private final PixyCam pixycam;
  private final DriveTrain drivetrain;
  private final PowercellSystem powercellsystem;

  public FindPathB(PixyCam pixycam, DriveTrain drivetrain, PowercellSystem powercellsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pixycam = pixycam;
    this.drivetrain = drivetrain;
    this.powercellsystem = powercellsystem;

    addRequirements(pixycam, drivetrain, powercellsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String path = pixycam.GetPath();
    if (path == "blue") {
      AutoPathBlueB currentPath = new AutoPathBlueB(drivetrain, powercellsystem);
      CommandScheduler.getInstance().schedule(currentPath);
    } else {
      if (path == "red") {
        AutoPathRedB currentPath = new AutoPathRedB(drivetrain, powercellsystem);
        CommandScheduler.getInstance().schedule(currentPath);
      }
    }
  }
}