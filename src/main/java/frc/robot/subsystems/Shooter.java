// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftPivotMotor;
  private CANSparkMax rightPivotMotor;
  //private SparkPIDController pivotPIDController;

  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;

  private CANSparkFlex feederAMPShooterMotor;

  private double desiredPivotAngle;

  private DigitalInput beamBreak;

  private double kG = 0.13107;

  /** Creates a new Shooter. */
  public Shooter() {
    leftPivotMotor = new CANSparkMax(ShooterConstants.leftPivotMotorID, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(ShooterConstants.rightPivotMotorID, MotorType.kBrushless);
    //pivotPIDController = rightPivotMotor.getPIDController();
    //pivotPIDController.setFeedbackDevice(pivotEncoder);


    topFlywheelMotor = new CANSparkFlex(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);

    feederAMPShooterMotor = new CANSparkFlex(ShooterConstants.feederAMPShooterMotorID, MotorType.kBrushless);

    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotor.restoreFactoryDefaults();
    topFlywheelMotor.restoreFactoryDefaults();
    bottomFlywheelMotor.restoreFactoryDefaults();
    feederAMPShooterMotor.restoreFactoryDefaults();

    //pivot config
    rightPivotMotor.getEncoder().setPositionConversionFactor(ShooterConstants.pivotPositionConversionFactor);
    rightPivotMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.pivotVelocityConversionFactor);
    leftPivotMotor.getEncoder().setPositionConversionFactor(ShooterConstants.pivotPositionConversionFactor);
    leftPivotMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.pivotVelocityConversionFactor);

    rightPivotMotor.setInverted(true);
    leftPivotMotor.follow(rightPivotMotor, true);

    /*pivotPIDController.setP(ShooterConstants.pivotPID_P, 0);
    pivotPIDController.setI(ShooterConstants.pivotPID_I, 0);
    pivotPIDController.setD(ShooterConstants.pivotPID_D, 0);
    pivotPIDController.setFF(ShooterConstants.pivotPID_FF, 0);
    pivotPIDController.setOutputRange(ShooterConstants.pivotPID_OutputMin, ShooterConstants.pivotPID_OutputMax);*/

    leftPivotMotor.setIdleMode(IdleMode.kBrake);
    rightPivotMotor.setIdleMode(IdleMode.kBrake);

    leftPivotMotor.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);
    rightPivotMotor.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);

    leftPivotMotor.burnFlash();
    rightPivotMotor.burnFlash();

    desiredPivotAngle = ShooterConstants.shooterRestingAngle;

    //flywheel config
    topFlywheelMotor.setInverted(true);
    bottomFlywheelMotor.setInverted(true);

    bottomFlywheelMotor.follow(topFlywheelMotor, true);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();

    //feeder&amp config
    feederAMPShooterMotor.setInverted(true);
    feederAMPShooterMotor.setIdleMode(IdleMode.kBrake);

    feederAMPShooterMotor.burnFlash();

    beamBreak = new DigitalInput(ShooterConstants.beamBreakPort);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    //pivotPIDController.setReference(desiredPivotAngle, ControlType.kPosition);

    SmartDashboard.putNumber("Pivot Angle", rightPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Follower Pivot Angle", leftPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Desired Pivot Angle", desiredPivotAngle);
  }

  public void setPivotAngle(double angle)
  {
    desiredPivotAngle = angle;
  }

  public boolean atDesiredAngle()
  {
    return Math.abs(rightPivotMotor.getEncoder().getPosition()-desiredPivotAngle) < ShooterConstants.shooterAngleTolerance;
  }

  public void setFlywheels(double speed)
  {
    topFlywheelMotor.set(speed);
  }

  public void setAMPFeeder(double speed)
  {
    feederAMPShooterMotor.set(speed);
  }

  public boolean getBeamBreak()
  {
    return beamBreak.get();
  }

  public void runVolts(double voltage)
  {
    rightPivotMotor.setVoltage(voltage/*+kG*Math.cos(Units.degreesToRadians(rightPivotMotor.getEncoder().getPosition()))*/);
  }

  public void stopVolts()
  {
    rightPivotMotor.setVoltage(0);
  }

  public boolean withinRange()
  {
    return rightPivotMotor.getEncoder().getPosition()>-5&&rightPivotMotor.getEncoder().getPosition()<85;
  }

  public void setEncoderPosition(double position)
  {
    rightPivotMotor.getEncoder().setPosition(position);
    leftPivotMotor.getEncoder().setPosition(position);
  }
}
