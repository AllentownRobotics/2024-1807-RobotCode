// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  CANSparkMax climbLeftMotor;
  CANSparkMax climbRightMotor;
  
  /** Creates a new Climb. */
  public Climb() {
    climbLeftMotor = new CANSparkMax(19,  MotorType.kBrushless);
    climbRightMotor = new CANSparkMax(20, MotorType.kBrushless);
    climbLeftMotor.setIdleMode(IdleMode.kBrake);
    climbRightMotor.setIdleMode(IdleMode.kBrake);
    climbRightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setLeftMotor(double power) {
    climbLeftMotor.set(power);
  }
  public void setRightMotor(double power) {
    climbRightMotor.set(power);
  }
}
