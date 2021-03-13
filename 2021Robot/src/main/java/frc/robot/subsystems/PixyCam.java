// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyCam extends SubsystemBase {
  /** Creates a new PixyCam. */

  //declare present value (ture or false)
  private boolean present = false;
  private Pixy2 pixycam;
  boolean isCamera = false;
  int state = -1;

  public PixyCam() {
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
  }

  public String GetPath() {
    String path = "";

    for (int count = 0; count < 300; count++) {
      if (!isCamera) {
        state = pixycam.init(1); // if no camera present, try to initialize
      }
      
      isCamera = state >= 0 ;
      SmartDashboard.putBoolean("Camera" , isCamera); 
      pixycam.getCCC().getBlocks(false , 255 , 255 ); 
      ArrayList<Block> blocks = pixycam.getCCC().getBlockCache();
  
      if (blocks.size() > 0 ) {
        double xcoord = blocks.get(0).getX();
        double ycoord = blocks.get(0).getY(); 
        String data = blocks.get(0).toString();
        SmartDashboard.putBoolean("present", true );
        SmartDashboard.putNumber("Xccord", xcoord);
        SmartDashboard.putNumber("Ycoord", ycoord);
        SmartDashboard.putString("Data", data );
        present = true;
      } else {
        SmartDashboard.putBoolean("present", false );
        present = false;
        SmartDashboard.putNumber("size", blocks.size());
      }
      if (present == false) {
        path = "blue";
      } else {
        return "red";
      } 
    }
    return path;
  }

//  @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     if (!isCamera) {
//       state = pixycam.init(1); // if no camera present, try to initialize
//     }
    
//     isCamera = state >= 0 ;
//     SmartDashboard.putBoolean("Camera" , isCamera); 
//     pixycam.getCCC().getBlocks(false , 255 , 255 ); 
//     ArrayList<Block> blocks = pixycam.getCCC().getBlockCache();

//     if (blocks.size() > 0 ) {
//       double xcoord = blocks.get(0).getX();
//       double ycoord = blocks.get(0).getY(); 
//       String data = blocks.get(0).toString();
//       SmartDashboard.putBoolean("present", true );
//       SmartDashboard.putNumber("Xccord", xcoord);
//       SmartDashboard.putNumber("Ycoord", ycoord);
//       SmartDashboard.putString("Data", data );
//     } else {
//       SmartDashboard.putBoolean("present", false );
//       SmartDashboard.putNumber("size", blocks.size());
//     }
//   }
  }