/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowercellSystem extends SubsystemBase {
  //private final Compressor compressor;
  private final DoubleSolenoid conveyorSolenoid;
  //private final DoubleSolenoid rearHatchSolenoid;
  // private final VictorSfinal final PX m_ConveyerMotor;
  private final WPI_VictorSPX conveyerMotor = new WPI_VictorSPX(Constants.CANID_ConveyerMotor);
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.CANID_IntakeMotor);
  private final AnalogInput pressureReader;

  private boolean armUp = false;

  public PowercellSystem() {
    //initializes compressor here
    conveyorSolenoid = new DoubleSolenoid (Constants.CANID_PCM, Constants.PCMID_ConveyerSoleniodForward, Constants.PCMID_ConveyerSoleniodBackward);
    Compressor compressor = new Compressor(Constants.CANID_PCM);
    pressureReader = new AnalogInput(0);
    compressor.setClosedLoopControl(true);
  }

  public boolean getArmUp(){
    return armUp;
  }

  public void setArmUp(boolean armUp){
    this.armUp = armUp;
    SmartDashboard.putBoolean("ArmUp", armUp);
  }

  public void RunConveyer(double speed) {
    conveyerMotor.set(speed);
  }

  public void RunIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void RaiseConveyer() {
    conveyorSolenoid.set(Value.kForward);
  }

  public void HoldConveyer() {
    conveyorSolenoid.set(Value.kOff); 
  }

  public void LowerConveyer() {
    conveyorSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("PSI Raw Voltage", pressureReader.getVoltage());
     SmartDashboard.putNumber("PSI", (pressureReader.getVoltage() - 1) * 36.25);
    //SmartDashboard.putNumber("Average PSI ", (pressureReader.getAverageVoltage() - 1) * 36.25);
  }
}