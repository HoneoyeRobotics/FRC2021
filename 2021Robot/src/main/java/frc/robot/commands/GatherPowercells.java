/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class GatherPowercells extends CommandBase {
  /**
   * Creates a new GatherPowercells.
   */
  private final PowercellSystem powerCellSystem;
  public GatherPowercells(PowercellSystem powerCellSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.powerCellSystem = powerCellSystem;
    addRequirements(powerCellSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(powerCellSystem.getArmUp() == true) {
      powerCellSystem.LowerConveyer();
      powerCellSystem.setArmUp(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powerCellSystem.RunIntake(1.0);
    powerCellSystem.RaiseConveyer();
    //powerCellSystem.CloseConveyerHatch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    powerCellSystem.RunIntake(0.0);
    powerCellSystem.HoldConveyer();
    //powerCellSystem.HoldConveyerHatch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}