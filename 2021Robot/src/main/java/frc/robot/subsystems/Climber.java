/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  
  private final DoubleSolenoid elevatorSolenoid;
  private final DoubleSolenoid extraLiftSolenoid;
  private boolean extraLiftEngaged = false;
  public Climber() {
    //initialize all motors here
    elevatorSolenoid = new DoubleSolenoid (Constants.CANID_PCM, Constants.PCMID_ScissorLiftForward, Constants.PCMID_ScissorLiftBackward);
    extraLiftSolenoid = new DoubleSolenoid (Constants.CANID_PCM, Constants.PCMID_ExtraLiftEngaged, Constants.PCMID_ExtraLiftDisengaged);
  }

  public boolean isExtraLiftEngaged(){
    return extraLiftEngaged;
  }
  
  public void EngageExtraLift() {
    extraLiftSolenoid.set(Value.kForward);
    extraLiftEngaged = true;
  }

  public void DisengageExtraLift() {
    extraLiftSolenoid.set(Value.kReverse);
    extraLiftEngaged = false;
  }

  public void HoldExtraLift() {
    extraLiftSolenoid.set(Value.kOff);
  }

  public void RaiseClimber() {
    elevatorSolenoid.set(Value.kForward);
  }

  public void LowerClimber() {
    elevatorSolenoid.set(Value.kReverse);
  }

  public void HoldClimber() {
    elevatorSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}