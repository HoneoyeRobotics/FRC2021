// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSquare extends SequentialCommandGroup {
  /** Creates a new AutoSquare. */
  
  private final DriveTrain m_drivetrain;
  public AutoSquare(DriveTrain drivetrain) {
    double waitTime = 0.25;
    double rotateTimeout = 1.5;
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrain = drivetrain;
    addCommands(
      new ResetOdometry(drivetrain),
    new AutoDriveForward(drivetrain, 50, 0.5),
    new WaitCommand(waitTime),
    new RotatePID(drivetrain, 90),
    new WaitCommand(waitTime),
    new AutoDriveForward(drivetrain,  50, 0.5),
    new WaitCommand(waitTime),
    new RotatePID(drivetrain, 180),
    new WaitCommand(waitTime),
    new AutoDriveForward(drivetrain,  50, 0.5),
    new WaitCommand(waitTime),
    new RotatePID(drivetrain, 270),
    new WaitCommand(waitTime),
    new AutoDriveForward(drivetrain,  50, 0.5));
    //new RotatePID(drivetrain, 0));
  }
}
