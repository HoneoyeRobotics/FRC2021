/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PowercellSystem;

public class RaiseConveyer extends CommandBase {
  private final PowercellSystem powercellSystem;
  public RaiseConveyer(PowercellSystem powercellSystem) {
    this.powercellSystem = powercellSystem;
    addRequirements(powercellSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    powercellSystem.RaiseConveyer();
    numTicks=0;
  }
  private int numTicks =0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    numTicks++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    powercellSystem.HoldConveyer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numTicks > 5;
  }
}
