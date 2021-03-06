// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.PowercellSystem;

public class FindPathA extends ConditionalCommand {

  public FindPathA(Command trueCommand, Command falseCommand, BooleanSupplier booleanSupplier) {
    super(trueCommand, falseCommand, booleanSupplier);
    //addRequirements(pixycam, drivetrain, powercellsystem);

  }

  // @Override
  // public void initialize() {
  //   String path = pixycam.GetPath();
  //   if (path == "blue") {
  //     AutoPathBlueA currentPath = ;
  //     currentPath.schedule();
  //   } else {
  //     if (path == "red") {
  //       ;
  //       currentPath.schedule();
  //     }
  //   }
  // }
}