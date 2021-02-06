/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import java.util.function.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * An example command that uses an example subsystem.
 */
public class StopDrives extends InstantCommand {
  private final DriveTrain m_drivetrain;

  public StopDrives( DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }
// Called once after isFinished returns true
@Override
public void end(boolean interrupted) {
  m_drivetrain.drive(0, 0);
}
}