/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PowercellSystem;

import java.util.ArrayList;
import java.util.function.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDriveRecord extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_zRotation;
  private final PowercellSystem m_powercellSystem;
  private final BooleanSupplier m_runArmWheel;

  public ArcadeDriveRecord(DoubleSupplier xSpeed, DoubleSupplier zRotation, DriveTrain drivetrain, PowercellSystem powercellSystem, BooleanSupplier runArmWheel) {
    m_drivetrain = drivetrain;
    m_xSpeed = xSpeed;
    m_zRotation = zRotation;
    m_powercellSystem = powercellSystem;
    m_runArmWheel = runArmWheel;
    addRequirements(m_drivetrain);
    addRequirements(powercellSystem);
  }
  private int tick;

  @Override
  public void initialize() {
    m_drivetrain.clearAutoDrive();
    tick = 0;
  }
  // Called repeatedly when this Command is scheduled to run
@Override
public void execute() {
  double xSpeed = m_xSpeed.getAsDouble();
  double zRotation = m_zRotation.getAsDouble() * 0.75;
  boolean runWheel = m_runArmWheel.getAsBoolean();
  m_drivetrain.drive(xSpeed,zRotation );
  m_drivetrain.addAutoDrive(new RecordedDrive(tick, xSpeed, zRotation,runWheel));
  if(runWheel)
    m_powercellSystem.RunIntake(0.5);
  else
    m_powercellSystem.RunIntake(0);
    tick++;
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
  m_powercellSystem.RunIntake(0);
  SmartDashboard.putNumber("AutoDriveRecords", m_drivetrain.autoDriveSize());
}
}