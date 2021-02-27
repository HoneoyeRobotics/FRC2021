// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PowercellSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathBlueA extends SequentialCommandGroup {
  /** Creates a new AutoSquare. */
  
  private final DriveTrain m_drivetrain;
  private final PowercellSystem m_powerCellSystem;

  public AutoPathBlueA(DriveTrain drivetrain, PowercellSystem powerCellSystem) {
    double waitTime = 0.25;
    double rotateTimeout = 1.5;
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrain = drivetrain;
    m_powerCellSystem = powerCellSystem;

    addCommands(
    new SequentialCommandGroup(
      // new WaitCommand(5),
      // new GatherPowercells(powerCellSystem)
      // new WaitCommand(6),
      // new GatherPowercells(powerCellSystem).withTimeout(5),
      // new WaitCommand(8),
      // new GatherPowercells(powerCellSystem).withTimeout(5)
    ),

    new SequentialCommandGroup(
      //new AutoDriveForward(drivetrain, distance, speed), will write distance in feet for now 
      //new RotatePID(drivetrain, angle), pos angle = right turn, neg angle = left turn
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 24),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 174.72, 0.5),
      new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -100),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 94.872, 0.5),
      new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 132),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 67.08, 0.5),
      new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 163.43),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 250, 0.5)));
      //reached point
  }
}
