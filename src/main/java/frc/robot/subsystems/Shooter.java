// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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
  private double desiredPivotAngle;
  
  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;
  private SparkPIDController topFlywheelPIDController;
  private SparkPIDController bottomFlywheelPIDController;

  private double desiredFlywheelRPM;

  private CANSparkFlex feederAMPShooterMotor;
    
  private DigitalInput limitSwitch;
  private DigitalInput beamBreakIndex;
  private DigitalInput beamBreakSource;

  private InterpolatingTreeMap<Double, Double> table;

  /** Creates a new Shooter. */
  public Shooter() {
    leftPivotMotor = new CANSparkMax(ShooterConstants.leftPivotMotorID, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(ShooterConstants.rightPivotMotorID, MotorType.kBrushless);
    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotor.restoreFactoryDefaults();
    
    pivotPIDController = rightPivotMotor.getPIDController();
    pivotPIDController.setFeedbackDevice(rightPivotMotor.getEncoder());
    armFeedforward = new ArmFeedforward(ShooterConstants.pivotFF_kS, ShooterConstants.pivotFF_kG, ShooterConstants.pivotFF_kV, ShooterConstants.pivotFF_kA);
    leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(ShooterConstants.pivotAbsoluteEncoderPositionConversionFactor);
    setEncoderPosition(90);
    
    topFlywheelMotor = new CANSparkFlex(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);
    topFlywheelMotor.restoreFactoryDefaults();
    bottomFlywheelMotor.restoreFactoryDefaults();


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
    rightPivotMotor.setSoftLimit(SoftLimitDirection.kForward, 130);
    rightPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 25);

    leftPivotMotor.burnFlash();
    rightPivotMotor.burnFlash();

    desiredPivotAngle = 54;

    //flywheel config
    topFlywheelMotor.setInverted(true);
    bottomFlywheelMotor.setInverted(true);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    topFlywheelPIDController = topFlywheelMotor.getPIDController();
    topFlywheelPIDController.setP(ShooterConstants.flywheelPID_P);
    topFlywheelPIDController.setI(ShooterConstants.flywheelPID_I);
    topFlywheelPIDController.setD(ShooterConstants.flywheelPID_D);
    topFlywheelPIDController.setFF(ShooterConstants.flywheelPID_FF);
    bottomFlywheelPIDController = bottomFlywheelMotor.getPIDController();
    bottomFlywheelPIDController.setP(ShooterConstants.flywheelPID_P);
    bottomFlywheelPIDController.setI(ShooterConstants.flywheelPID_I);
    bottomFlywheelPIDController.setD(ShooterConstants.flywheelPID_D);
    bottomFlywheelPIDController.setFF(ShooterConstants.flywheelPID_FF);

    topFlywheelMotor.getEncoder().setPositionConversionFactor(ShooterConstants.flywheelPositionConversionFactor);
    topFlywheelMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.flywheelVelocityConversionFactor);
    bottomFlywheelMotor.getEncoder().setPositionConversionFactor(ShooterConstants.flywheelPositionConversionFactor);
    bottomFlywheelMotor.getEncoder().setVelocityConversionFactor(ShooterConstants.flywheelVelocityConversionFactor);

    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();

    desiredFlywheelRPM = 0;

    //feeder&amp config
    feederAMPShooterMotor = new CANSparkFlex(ShooterConstants.feederAMPShooterMotorID, MotorType.kBrushless);
    feederAMPShooterMotor.restoreFactoryDefaults();
    
    feederAMPShooterMotor.setInverted(false);
    feederAMPShooterMotor.setIdleMode(IdleMode.kBrake);

    feederAMPShooterMotor.burnFlash();

    //sensor config
    limitSwitch = new DigitalInput(ShooterConstants.limitSwitchPort);
    beamBreakIndex = new DigitalInput(ShooterConstants.beamBreakIndexPort);
    beamBreakSource = new DigitalInput(ShooterConstants.beamBreakSourcePort);

    //interpolation set up
    table =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    /*
    old data
    table.put(1.37, 57.0);
    table.put(2.0, 45.0);
    table.put(3.0, 37.0);
    table.put(3.5, 32.0);
    table.put(3.75, 29.0);
    table.put(4.0, 28.0);
    table.put(4.2, 28.0);
    table.put(4.3, 27.0);*/

    /*table.put(1.44, 54.0);
    table.put(2.0, 43.0);
    table.put(2.5, 39.0);
    table.put(3.0, 35.0);
    table.put(3.25, 34.0);
    table.put(3.5, 33.0);
    table.put(3.75, 31.0);
    table.put(4.0, 30.0);
    table.put(4.25, 27.0);*/

    table.put(1.4, 54.0);
    table.put(1.75, 47.0);
    table.put(2.0, 42.0);
    table.put(2.5, 38.0);
    table.put(3.0, 33.0);
    table.put(3.25, 32.0);
    table.put(3.5, 30.0);
    table.put(3.75, 29.0);
    table.put(4.0, 28.0);
    table.put(4.25, 27.0);
    table.put(4.5, 26.5);
    table.put(4.75, 26.0);
  }

  @Override
  public void periodic() 
  {
    //pivot periodic monitoring
    pivotPIDController.setReference(desiredPivotAngle, ControlType.kPosition,0, armFeedforward.calculate(Units.degreesToRadians(desiredPivotAngle), 0), ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("Pivot Angle", rightPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Desired Pivot Angle", desiredPivotAngle);

    //flywheels periodic monitoring
    SmartDashboard.putNumber("desired RPM", desiredFlywheelRPM);
    SmartDashboard.putNumber("top flywheel speed", topFlywheelMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("bottom flywheel speed", bottomFlywheelMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("Absol encoder pos", leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
  }

  public void setPivotAngle(double angle)
  {
    desiredPivotAngle = angle;
  }

  public boolean atDesiredAngle()
  {
    return Math.abs(rightPivotMotor.getEncoder().getPosition()-desiredPivotAngle) < ShooterConstants.shooterAngleTolerance;
  }

  public void setFlywheelsRPM(double rpm)
  {
    topFlywheelPIDController.setReference(rpm, ControlType.kVelocity);
    bottomFlywheelPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setFlywheelsPercent(double percent)
  {
    topFlywheelPIDController.setReference(percent, ControlType.kDutyCycle);
    bottomFlywheelPIDController.setReference(percent, ControlType.kDutyCycle);
  }

  public boolean atDesiredRPM()
  {
    return Math.abs(topFlywheelMotor.getEncoder().getVelocity()-ShooterConstants.shootingRPM) < ShooterConstants.shootingRPMTolerance;
  }

  public boolean atDesiredRPMTrap()
  {
    return Math.abs(topFlywheelMotor.getEncoder().getVelocity()-ShooterConstants.shootingRPM) < ShooterConstants.shootingRPMTolerance;
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

  public void setEncoderPosition(double position)
  {
    rightPivotMotor.getEncoder().setPosition(90-(ShooterConstants.pivotAbsoluteEncoder90-leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
    leftPivotMotor.getEncoder().setPosition(90-(ShooterConstants.pivotAbsoluteEncoder90-leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
  }

  public double getAimingAngle(double distance)
  {
    return table.get(distance);
  }

  public void incrementSetpoit(double increment)
  {
    desiredPivotAngle += increment;
  }

  public boolean getLimitSwitch()
  {
    return limitSwitch.get();
  }
}
