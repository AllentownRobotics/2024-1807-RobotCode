// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftPivotMotor;
  private CANSparkMax rightPivotMotor;
  private AbsoluteEncoder pivotEncoder;
  private SparkPIDController pivotPIDController;

  private CANSparkFlex topShooterMotor;
  private CANSparkFlex bottomShooterMotor;

  private CANSparkMax feederAMPShooterMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    leftPivotMotor = new CANSparkMax(ShooterConstants.leftPivotMotorID, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(ShooterConstants.rightPivotMotorID, MotorType.kBrushless);
    pivotEncoder = rightPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotPIDController = rightPivotMotor.getPIDController();


    topShooterMotor = new CANSparkFlex(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkFlex(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);

    feederAMPShooterMotor = new CANSparkMax(ShooterConstants.feederAMPShooterMotorID, MotorType.kBrushless);

    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotor.restoreFactoryDefaults();
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();
    feederAMPShooterMotor.restoreFactoryDefaults();

    pivotEncoder.setInverted(false);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.pivotPositionConversionFactor);

    rightPivotMotor.setInverted(false);
    leftPivotMotor.follow(rightPivotMotor, true);

    pivotPIDController.setP(ShooterConstants.pivotPID_P, 0);
    pivotPIDController.setI(ShooterConstants.pivotPID_I, 0);
    pivotPIDController.setD(ShooterConstants.pivotPID_D, 0);
    pivotPIDController.setFF(ShooterConstants.pivotPID_FF, 0);
    pivotPIDController.setOutputRange(ShooterConstants.pivotPID_OutputMin, ShooterConstants.pivotPID_OutputMax);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
