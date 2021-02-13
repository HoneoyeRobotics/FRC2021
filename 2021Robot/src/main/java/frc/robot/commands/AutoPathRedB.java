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
public class AutoPathRedB extends SequentialCommandGroup {
  /** Creates a new AutoSquare. */
  
  private final DriveTrain m_drivetrain;
  public AutoPathRedB(DriveTrain drivetrain) {
    double waitTime = 0.25;
    double rotateTimeout = 1.5;
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrain = drivetrain;
    addCommands(
      //new AutoDriveForward(drivetrain, distance, speed), will write distance in feet for now 
      //new RotatePID(drivetrain, angle), pos angle = right turn, neg angle = left turn
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -18.435),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 7.906, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 71.565),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 7.071, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -90),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 7.071, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -106.26),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 17.678, 0.5));
      //reached point
  }
}
