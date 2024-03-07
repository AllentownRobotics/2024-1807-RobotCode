// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  CANSparkMax leftClimbMotor;
  CANSparkMax rightClimbMotor;

  DigitalInput leftClimbLimitSwitch;
  DigitalInput rightClimbLimitSwitch;
  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor = new CANSparkMax(ClimbConstants.leftClimbMotorID, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(ClimbConstants.rightClimbMotorID, MotorType.kBrushless);

    leftClimbMotor.setInverted(false);
    rightClimbMotor.setInverted(true);

    leftClimbLimitSwitch = new DigitalInput(ClimbConstants.leftClimbLimitSwitch);
    rightClimbLimitSwitch = new DigitalInput(ClimbConstants.rightClimbLimitSwitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("left Climb", leftClimbLimitSwitch.get());
    SmartDashboard.putBoolean("right climb", rightClimbLimitSwitch.get());
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

  public boolean getLeftLimit()
  {
    return leftClimbLimitSwitch.get();
  }

  public boolean getRightLimit()
  {
    return rightClimbLimitSwitch.get();
  }

  public void zeroLeftEncoder()
  {
    leftClimbMotor.getEncoder().setPosition(0);
    leftClimbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimbMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
  }

  public void zeroRightEncoder()
  {
    rightClimbMotor.getEncoder().setPosition(0);
    rightClimbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimbMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
  }
}
