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

  boolean shooterReady;
  boolean alignReady;

  /** Creates a new Limelight. */
  public Vision() {
    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    frontLimelightTable.getEntry("priorityid").setNumber(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    z = Math.abs(frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2]);
    x = Math.abs(frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0]);

    SmartDashboard.putBoolean("align Ready", facingSpeaker());
    SmartDashboard.putBoolean("shooter ready", shooterReady);

    if(shooterReady&&facingSpeaker())
    {
      frontLightOn();
    }
    else
    {
      frontLightOff();
    }
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

  public void frontLightOn()
  {
    frontLimelightTable.getEntry("ledMode").setNumber(3);
  }

  public void frontLightOff()
  {
    frontLimelightTable.getEntry("ledMode").setNumber(1);
  }

  public void setShooterReady(boolean bool)
  {
    shooterReady = bool;
  }

  public boolean getShooterReady()
  {
    return shooterReady;
  }

  public void alignReady(boolean bool)
  {
    alignReady = bool;
  }
}