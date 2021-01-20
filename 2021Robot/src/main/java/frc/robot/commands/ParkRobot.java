/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ParkDirection;
import frc.robot.subsystems.DriveTrain;

import java.util.function.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ParkRobot extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final ParkDirection parkDirection;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ParkRobot(DriveTrain drivetrain, ParkDirection parkDirection) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    this.parkDirection = parkDirection;
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_drivetrain.drive(Constants.autoParkFwdSpeed, Constants.autoParkRotateSpeed * (parkDirection == ParkDirection.Left ? -1 : 1));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}