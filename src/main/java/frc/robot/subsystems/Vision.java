// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  NetworkTable frontLimelightTable;
  NetworkTable rearLimelightTable;

  double x;
  double z;

  /** Creates a new Limelight. */
  public Vision() {
    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    frontLimelightTable.getEntry("pipeline").setNumber(0);
    if(DriverStation.getAlliance().equals(Alliance.Blue))
    {
      frontLimelightTable.getEntry("priorityid").setNumber(7);
    }
    else
    {
      frontLimelightTable.getEntry("priorityid").setNumber(4);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    z = Math.abs(frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2]);
    x = Math.abs(frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0]);

    SmartDashboard.putNumber("THIS ONE: distance to speaker", getDistanceToSpeaker());
    SmartDashboard.putNumber("degrees to speaker", getDegreesToSpeaker());
  }

  public double getDegreesToSpeaker()
  {
    return frontLimelightTable.getEntry("tx").getDouble(0);
  }

  public double getDistanceToSpeaker()
  {
    return Math.sqrt(Math.pow(z, 2)+Math.pow(x,2));
  }

  public boolean facingSpeaker()
  {
    return Math.abs(getDegreesToSpeaker())<VisionConstants.rotationTolerance;
  }
}