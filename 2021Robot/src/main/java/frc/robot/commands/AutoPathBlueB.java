// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PowercellSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathBlueB extends ParallelCommandGroup {
  /** Creates a new AutoSquare. */
  
  private final DriveTrain m_drivetrain;
  private final PowercellSystem m_powerCellSystem;

  public AutoPathBlueB(DriveTrain drivetrain, PowercellSystem powerCellSystem) {
    double waitTime = 0.25;
    double rotateTimeout = 1.5;
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drivetrain = drivetrain;
    m_powerCellSystem = powerCellSystem;

    addCommands(
      new SequentialCommandGroup(
        new LowerConveyer(powerCellSystem),
        new GatherPowercells(powerCellSystem)
      ),

    new SequentialCommandGroup(
      //new AutoDriveForward(drivetrain, distance, speed), will write distance in feet for now 
      //new RotatePID(drivetrain, angle), pos angle = right turn, neg angle = left turn
      //new LowerConveyer(powerCellSystem),
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 25).withTimeout(5),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 120, 0.5),
      new WaitCommand(waitTime),
      //reached point
      //needs to be messed with
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -70).withTimeout(5),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 100, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 95).withTimeout(5),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 85, 0.5),
      new WaitCommand(waitTime)));
      //reached point
      // new ResetOdometry(drivetrain),
      // new RotatePID(drivetrain, -35).withTimeout(1),
      // new WaitCommand(waitTime), 
      // new AutoDriveForward(drivetrain, 36, 0.5)));
      //reached point
  }
}
