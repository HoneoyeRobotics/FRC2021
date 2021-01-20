/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PowercellSystem;

public class DeployArm extends CommandBase {
  private final PowercellSystem powercellSystem;
  private Timer timer = new Timer();

  public DeployArm(PowercellSystem powercellSystem) {
    this.powercellSystem = powercellSystem;
    addRequirements(powercellSystem);
  }

  // Called when the command is initially scheduled.
  private boolean done = false;

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (powercellSystem.getArmUp() == true) {
      done = true;
    } else {
      done = false;
      powercellSystem.LowerConveyer();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= 0.25) {
      powercellSystem.RaiseConveyer();
    } else if (timer.get() >= 0.5) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    powercellSystem.HoldConveyer();
    powercellSystem.setArmUp(false);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
