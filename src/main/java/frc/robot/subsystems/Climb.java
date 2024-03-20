// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  CANSparkMax leftClimbMotor;
  CANSparkMax rightClimbMotor;
  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor = new CANSparkMax(ClimbConstants.leftClimbMotorID, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(ClimbConstants.rightClimbMotorID, MotorType.kBrushless);

    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);

    leftClimbMotor.setInverted(false);
    rightClimbMotor.setInverted(true);

    leftClimbMotor.getEncoder().setPosition(0);
    rightClimbMotor.getEncoder().setPosition(0);

    leftClimbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimbMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    rightClimbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimbMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    leftClimbMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    rightClimbMotor.setSoftLimit(SoftLimitDirection.kForward, 0);

    leftClimbMotor.setSmartCurrentLimit(38);
    rightClimbMotor.setSmartCurrentLimit(38);
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
