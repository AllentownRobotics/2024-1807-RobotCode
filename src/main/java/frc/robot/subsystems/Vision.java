// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.aruco.EstimateParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  NetworkTable frontLimelightTable;
  NetworkTable rearLimelightTable;

  Pose2d frontLimelightBotPose;
  double frontLimelightNumberOfTags;
  double frontLimelightAverageDistanceToTags;
  double frontLimelightLatency;
  double frontLimelightAmbiguity;

  Pose2d rearLimelightBotPose;
  double rearLimelightNumberOfTags;
  double rearLimelightAverageDistanceToTags;
  double rearLimelightLatency;
  double rearLimelightAmbiguity;

  double x;
  double z;

  /** Creates a new Limelight. */
  public Vision() {
    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    rearLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-rear");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLimelightBotPose = new Pose2d(frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0], frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[1], new Rotation2d(frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[5]));
    frontLimelightNumberOfTags = frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[7];
    frontLimelightLatency = frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[6]/1000;
    frontLimelightAverageDistanceToTags = frontLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[9];

    rearLimelightBotPose = new Pose2d(rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0], rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[1], new Rotation2d(rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[5]));
    rearLimelightNumberOfTags = rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[7];
    rearLimelightLatency = rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[6]/1000;
    rearLimelightAverageDistanceToTags = rearLimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6])[9];

    updateVisionMeasurement();
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

  public void updateVisionMeasurement()
  {
    if(frontLimelightAverageDistanceToTags<VisionConstants.maxDistance)
    {
      if(frontLimelightNumberOfTags >= VisionConstants.minNumberOfTags)
      {
        DriveTrain.addVisionMeasurement(frontLimelightBotPose,frontLimelightLatency);
      }
      else if(frontLimelightAmbiguity < VisionConstants.maxSingleTagAmbiguity)
      {
        DriveTrain.addVisionMeasurement(frontLimelightBotPose, frontLimelightLatency);
      }
    }
  }
}