/*----------------------------------------------------------------------------*/
/* CopyzRotation (c) 2018-2019 FIRST. All zRotations Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PowercellSystem;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.function.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ArcadeDrivePlay extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final PowercellSystem m_powercellSystem;

  public ArcadeDrivePlay(DriveTrain drivetrain, PowercellSystem powercellSystem) {
    m_drivetrain = drivetrain;
    m_powercellSystem = powercellSystem;
    addRequirements(m_drivetrain);
    addRequirements(powercellSystem);
  }

  private int tick;
  private int stop;

  @Override
  public void initialize() {
   
    tick =0;

    stop =  m_drivetrain.autoDriveSize();
  }
  // Called repeatedly when this Command is scheduled to run
@Override
public void execute() {

  RecordedDrive current = m_drivetrain.getAutoDrive(tick);
  
  m_drivetrain.drive(current.xSpeed,current.zRotation );
  if(current.runwheel)
    m_powercellSystem.RunIntake(0.5);
  else
    m_powercellSystem.RunIntake(0);
  tick++;
}

//Make this return true when this Command no longer needs to run execute()
@Override
public boolean isFinished() {
  
  return tick >= stop;
}
	
File f;
BufferedWriter bw;
FileWriter fw;

// Called once after isFinished returns true
@Override
public void end(boolean interrupted) {
  m_drivetrain.drive(0, 0);
  m_powercellSystem.RunIntake(0);

  SmartDashboard.putNumber("AutoDriveRecords", m_drivetrain.autoDriveSize());
}
}