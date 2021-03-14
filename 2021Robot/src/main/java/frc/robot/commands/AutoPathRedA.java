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
public class AutoPathRedA extends ParallelCommandGroup {
  /** Creates a new AutoSquare. */
  
  
  public AutoPathRedA(DriveTrain drivetrain, PowercellSystem powerCellSystem,
    double waitTime,
    double rotateTimeout){
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
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
      new WaitCommand(waitTime),
      new AutoDriveForward(drivetrain, 70, 0.5),
      //new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 80).withTimeout(5),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 70, 0.5),
      //new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -146).withTimeout(5),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 130, 0.5),
      //new GatherPowercells(powerCellSystem).withTimeout(1),
      new WaitCommand(waitTime),
      //reached point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 50).withTimeout(1),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 210, 0.5)));
      //reached point
  }
}
