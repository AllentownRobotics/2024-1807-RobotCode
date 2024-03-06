// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTable frontLimelightTable;
  NetworkTable rearLimelightTable;

  /** Creates a new Limelight. */
  public Vision() {
    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    frontLimelightTable.getEntry("pipeline").setNumber(0);
    SmartDashboard.putNumber("test limelight", getX());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getX()
  {
    return frontLimelightTable.getEntry("tx").getDouble(0);
  }
}