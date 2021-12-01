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
public class AutoBouncePad extends ParallelCommandGroup {
  /** Creates a new AutoSquare. */
  
  
  public AutoBouncePad(DriveTrain drivetrain, PowercellSystem powerCellSystem,
    double waitTime,
    double rotateTimeout){
    //double rotateAngle = 90;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // new SequentialCommandGroup(
    //   new LowerConveyer(powerCellSystem),
    //   new GatherPowercells(powerCellSystem)
    // ),

    new SequentialCommandGroup(
      //new AutoDriveForward(drivetrain, distance, speed), distance is in inches
      //new RotatePID(drivetrain, angle), pos angle = right turn, neg angle = left turn
      //new LowerConveyer(powerCellSystem),

      //at start, rotate toward ball and then drive to hit
      new ResetOdometry(drivetrain),
      new AutoDriveForward(drivetrain, 25, 0.4),
      new WaitCommand(waitTime),
      new RotatePID(drivetrain, -90).withTimeout(4),
      new WaitCommand(waitTime),
      new AutoDriveForward(drivetrain, 35, 0.4),
      new WaitCommand(waitTime),
      
      //reached point, rotate and drive toward line
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -20).withTimeout(4), 
      new WaitCommand(waitTime), 
      new AutoDriveReverse(drivetrain, 130, 0.4),
      new WaitCommand(waitTime),
      
      //reached point, rotate on line and go to ball point
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -70).withTimeout(4), // figure out
      new WaitCommand(waitTime), 
      new AutoDriveReverse(drivetrain, 25, 0.4),
      new WaitCommand(waitTime),

      //reached point, rotate toward ball and hit
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 90).withTimeout(4),
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 120, 0.4),
      new WaitCommand(waitTime),

      //reached point, return and rotate back 
      new AutoDriveReverse(drivetrain, 110, 0.35),
      new WaitCommand(waitTime),
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 90).withTimeout(4),
      
      //reached point, drive forward and rotate toward ball
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 70, 0.4),
      new WaitCommand(waitTime),
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, -90).withTimeout(4),
      
      //reached point, drive toward ball, drive back
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 120, 0.4),
      new WaitCommand(waitTime),
      new AutoDriveReverse(drivetrain, 60, 0.35),
      
      //reach point, rotate toward end, finish
      new ResetOdometry(drivetrain),
      new RotatePID(drivetrain, 90).withTimeout(3), //final turn, leave it alone%
      new WaitCommand(waitTime), 
      new AutoDriveForward(drivetrain, 120, 0.4)));
  }
}
