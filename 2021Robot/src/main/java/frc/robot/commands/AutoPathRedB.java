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
public class AutoPathRedB extends ParallelCommandGroup {
  /** Creates a new AutoSquare. */
  
  private final DriveTrain m_drivetrain;
  private final PowercellSystem m_powerCellSystem;
  public AutoPathRedB(DriveTrain drivetrain, PowercellSystem powerCellSystem) {
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
      new LowerConveyer(powerCellSystem),
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -18.435).withTimeout(7),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 50, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 73).withTimeout(7),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 80, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -98).withTimeout(7),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 90, 0.5),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      //need to recalc angle
      new RotatePID(drivetrain, 45).withTimeout(1),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 100, 0.5)));
      //reached point
  }
}
