// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private CANSparkMax frontIndexerMotor;
  private CANSparkMax rearIndexerMotor;

  /** Creates a new Indexer. */
  public Indexer() {
    //indexer Config
    frontIndexerMotor = new CANSparkMax(IndexerConstants.frontIndexMotorID, MotorType.kBrushless);
    rearIndexerMotor = new CANSparkMax(IndexerConstants.rearIndexMotorID, MotorType.kBrushless);

    frontIndexerMotor.restoreFactoryDefaults();
    rearIndexerMotor.restoreFactoryDefaults();

    frontIndexerMotor.setIdleMode(IdleMode.kBrake);
    rearIndexerMotor.setIdleMode(IdleMode.kBrake);

    frontIndexerMotor.setInverted(true);
    rearIndexerMotor.follow(frontIndexerMotor, true);

    frontIndexerMotor.burnFlash();
    rearIndexerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void index(double speed)
  {
    frontIndexerMotor.set(speed);
  }
}
