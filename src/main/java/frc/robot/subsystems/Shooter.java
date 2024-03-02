// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax leftPivotMotor;
  private CANSparkMax rightPivotMotor;
  private SparkPIDController pivotPIDController;
  private ArmFeedforward armFeedforward;

  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;
  private BangBangController flywheelController;
  private double desiredFlywheelRPM;

  private CANSparkFlex feederAMPShooterMotor;

  private double desiredPivotAngle;

  private DigitalInput beamBreakIndex;
  private DigitalInput beamBreakSource;

  private double kG = 0.12218;

  /** Creates a new Shooter. */
  public Shooter() {
    leftPivotMotor = new CANSparkMax(ShooterConstants.leftPivotMotorID, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(ShooterConstants.rightPivotMotorID, MotorType.kBrushless);
    pivotPIDController = rightPivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(rightPivotMotor.getEncoder());
    armFeedforward = new ArmFeedforward(ShooterConstants.pivotFF_kS, ShooterConstants.pivotFF_kG, ShooterConstants.pivotFF_kV, ShooterConstants.pivotFF_kA);


    topFlywheelMotor = new CANSparkFlex(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);

    feederAMPShooterMotor = new CANSparkFlex(ShooterConstants.feederAMPShooterMotorID, MotorType.kBrushless);

    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotor.restoreFactoryDefaults();
    topFlywheelMotor.restoreFactoryDefaults();
    bottomFlywheelMotor.restoreFactoryDefaults();
    feederAMPShooterMotor.restoreFactoryDefaults();

    //pivot config
    rightPivotMotor.getEncoder().setPositionConversionFactor(ShooterConstants.pivotMotorPositionConversionFactor);
    rightPivotMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.pivotVelocityConversionFactor);
    leftPivotMotor.getEncoder().setPositionConversionFactor(ShooterConstants.pivotMotorPositionConversionFactor);
    leftPivotMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.pivotVelocityConversionFactor);

    rightPivotMotor.setInverted(false);
    leftPivotMotor.follow(rightPivotMotor, true);

    pivotPIDController.setP(ShooterConstants.pivotPID_P, 0);
    pivotPIDController.setI(ShooterConstants.pivotPID_I, 0);
    pivotPIDController.setD(ShooterConstants.pivotPID_D, 0);
    pivotPIDController.setFF(ShooterConstants.pivotPID_FF, 0);
    pivotPIDController.setOutputRange(ShooterConstants.pivotPID_OutputMin, ShooterConstants.pivotPID_OutputMax);

    leftPivotMotor.setIdleMode(IdleMode.kBrake);
    rightPivotMotor.setIdleMode(IdleMode.kBrake);

    leftPivotMotor.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);
    rightPivotMotor.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);

    rightPivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightPivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    rightPivotMotor.setSoftLimit(SoftLimitDirection.kForward, 110);
    rightPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 40);


    leftPivotMotor.burnFlash();
    rightPivotMotor.burnFlash();

    desiredPivotAngle = ShooterConstants.shooterRestingAngle;

    //flywheel config
    topFlywheelMotor.setInverted(true);
    bottomFlywheelMotor.setInverted(true);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();

    //feeder&amp config
    feederAMPShooterMotor.setInverted(false);
    feederAMPShooterMotor.setIdleMode(IdleMode.kBrake);

    feederAMPShooterMotor.burnFlash();

    beamBreakIndex = new DigitalInput(ShooterConstants.beamBreakIndexPort);
    beamBreakSource = new DigitalInput(ShooterConstants.beamBreakSourcePort);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    pivotPIDController.setReference(desiredPivotAngle, ControlType.kPosition,0, armFeedforward.calculate(Units.degreesToRadians(desiredPivotAngle), 0), ArbFFUnits.kVoltage);

    SmartDashboard.putNumber("Pivot Angle", rightPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Follower Pivot Angle", leftPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Desired Pivot Angle", desiredPivotAngle);

    SmartDashboard.putNumber("top flywheel speed", topFlywheelMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("bottom flywheel speed", bottomFlywheelMotor.getEncoder().getVelocity());
  }

  public void setPivotAngle(double angle)
  {
    desiredPivotAngle = angle;
  }

  public boolean atDesiredAngle()
  {
    return Math.abs(rightPivotMotor.getEncoder().getPosition()-desiredPivotAngle) < ShooterConstants.shooterAngleTolerance;
  }

  public void runFlywheelsShooting(double speed)
  {
    flywheelController.setSetpoint(speed);
    topFlywheelMotor.set(flywheelController.calculate(topFlywheelMotor.getEncoder().getVelocity()));
    bottomFlywheelMotor.set(flywheelController.calculate(bottomFlywheelMotor.getEncoder().getVelocity()));
  }

  public boolean shootingFlywheelsAtRPM()
  {
    return flywheelController.atSetpoint();
  }

  public void setFlywheelsShooting(double speed)
  {
    topFlywheelMotor.set(speed);
    bottomFlywheelMotor.set(speed);
  }

  public void setFlywheelsAMPing(double speed)
  {
    topFlywheelMotor.set(speed);
    bottomFlywheelMotor.set(-speed);
  }

  public void setAMPFeeder(double speed)
  {
    feederAMPShooterMotor.set(speed);
  }

  /**Returns state of the beam break for the ground collection in the shooter
   * @return True if beam break is broken, false otherwise
   */
  public boolean getBeamBreakIndex()
  {
    return !beamBreakIndex.get();
  }

  /**Returns state of the beam break for the source collection in the shooter
   * @return True if beam break is broken, false otherwise
   */
  public boolean getBeamBreakSource()
  {
    return !beamBreakSource.get();
  }

  public void runVolts(double voltage)
  {
    rightPivotMotor.setVoltage(voltage+kG*Math.cos(Units.degreesToRadians(rightPivotMotor.getEncoder().getPosition())));
  }

  public void stopVolts()
  {
    rightPivotMotor.setVoltage(0);
  }

  public boolean withinRange()
  {
    return rightPivotMotor.getEncoder().getPosition()>-5&&rightPivotMotor.getEncoder().getPosition()<85;
  }

  public boolean atUpperThreshold()
  {
    return rightPivotMotor.getEncoder().getPosition()>110;
  }

  public boolean atLowerThreshold()
  {
    return rightPivotMotor.getEncoder().getPosition()<40;
  }

  public void setEncoderPosition(double position)
  {
    rightPivotMotor.getEncoder().setPosition(position);
    leftPivotMotor.getEncoder().setPosition(position);
  }

  public double getAimingAngle(double distance)
  {
    return distance;
  }
}
