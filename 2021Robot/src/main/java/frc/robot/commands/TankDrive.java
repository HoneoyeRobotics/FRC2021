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

/**
 * An example command that uses an example subsystem.
 */
public class TankDrive extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_Left;
  private final DoubleSupplier m_Right;

  public TankDrive(DoubleSupplier left, DoubleSupplier right, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    m_Left = left;
    m_Right = right;
    addRequirements(m_drivetrain);
  }

// Called repeatedly when this Command is scheduled to run
@Override
public void execute() {
 // m_drivetrain.tankDrive(m_Left.getAsDouble(), m_Right.getAsDouble());

 m_drivetrain.drive(m_Left.getAsDouble(), m_Right.getAsDouble());
}

//Make this return true when this Command no longer needs to run execute()
@Override
public boolean isFinished() {
  return false; // Runs until interrupted
}

// Called once after isFinished returns true
@Override
public void end(boolean interrupted) {
  m_drivetrain.drive(0, 0);
}
}