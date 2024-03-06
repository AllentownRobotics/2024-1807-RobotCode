// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  CANSparkMax leftClimbMotor;
  CANSparkMax rightClimbMotor;
  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor = new CANSparkMax(19, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(20, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimb(double speed)
  {
    leftClimbMotor.set(speed);
    rightClimbMotor.set(speed);
  }

  public void setLeft(double speed)
  {
    leftClimbMotor.set(speed);
  }

  public void setRight(double speed)
  {
    rightClimbMotor.set(speed);
  }
}
