// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
  private final CANSparkMax turningSparkMax;
  private final TalonFX driveKraken;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.14382, 2.4001, 0.39733);
  //private final CANSparkFlex driveSparkFlex;
 
  //private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

public SwerveModule(int driveID, int turningID, double chassisAngularOffset) {

  //set up swerve modules
  //driveSparkFlex = new CANSparkFlex(driveID, MotorType.kBrushless);
  driveKraken = new TalonFX(driveID);
  /*TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
  krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  krakenConfig.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_ENCODER_POS_FACTOR;
  krakenConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  krakenConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT;
  krakenConfig.CurrentLimits.SupplyCurrentThreshold = 60;
  krakenConfig.CurrentLimits.SupplyTimeThreshold = .1;
  krakenConfig.Slot0.kP = 0.005;
  krakenConfig.Slot0.kI = 0.0;
  krakenConfig.Slot0.kD = 0.0;
  krakenConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ModuleConstants.OPEN_LOOP_RAMP_RATE;
  krakenConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.CLOSED_LOOP_RAMP_RATE;
  var slot0Configs = new Slot0Configs();
  slot0Configs.kP = ModuleConstants.DRIVE_P;
  slot0Configs.kI = ModuleConstants.DRIVE_I;
  slot0Configs.kD = ModuleConstants.DRIVE_D;
  driveKraken.getConfigurator().apply(slot0Configs);*/
  TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 5.08/(0.076*Math.PI);
    config.Slot0.kP = 0.05;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  driveKraken.getConfigurator().apply(config);
  driveKraken.getConfigurator().setPosition(0.0);
  turningSparkMax = new CANSparkMax(turningID, MotorType.kBrushless);
  
  //driveSparkFlex.restoreFactoryDefaults();
  turningSparkMax.restoreFactoryDefaults(); 

  //driveEncoder = driveSparkFlex.getEncoder();
  turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);


  turningPIDController = turningSparkMax.getPIDController();

  turningPIDController.setFeedbackDevice(turningEncoder);

  //driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POS_FACTOR);
  //driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR); 
  turningEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POS_FACTOR);
  turningEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

  turningEncoder.setInverted(ModuleConstants.invertTurnEncoder);

  turningPIDController.setPositionPIDWrappingEnabled(true);
  turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_ENCODER_POS_MAX_INPUT);
  turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_ENCODER_POS_MIN_INPUT);
  
  //set drive motor PID values
  

  //Set turn motor PID values 
  turningPIDController.setP(ModuleConstants.TURN_P);
  turningPIDController.setI(ModuleConstants.TURN_I);
  turningPIDController.setD(ModuleConstants.TURN_D);
  turningPIDController.setFF(ModuleConstants.TURN_FF);
  turningPIDController.setOutputRange(ModuleConstants.TURN_MIN_OUTPUT, ModuleConstants.TURN_MAX_OUTPUT);
  
  //limits and brakes
  //driveSparkFlex.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
  driveKraken.setNeutralMode(NeutralModeValue.Brake);
  turningSparkMax.setIdleMode(ModuleConstants.TURN_MOTOR_IDLE_MODE); 
  //driveSparkFlex.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
  turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

  //saves configs of modules in case of brownout
  //driveSparkFlex.burnFlash();
  turningSparkMax.burnFlash();

  this.chassisAngularOffset = chassisAngularOffset;
  desiredState.angle = new Rotation2d(turningEncoder.getPosition());
  //driveEncoder.setPosition(0);
}

//returns the current state of the module
public SwerveModuleState getState(){
  return new SwerveModuleState(driveKraken.getVelocity().getValue(), 
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
}

//returns current position of the module 
public SwerveModulePosition getPosition(){
  return new SwerveModulePosition(
    driveKraken.getPosition().getValue(),
    new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
}

//sets desired state for module 
public void setDesiredState(SwerveModuleState swerveModuleStates){

  //apply chassis angular offset to desired state 
  SwerveModuleState correctedDesiredState = new SwerveModuleState();
  correctedDesiredState.speedMetersPerSecond = swerveModuleStates.speedMetersPerSecond;
  correctedDesiredState.angle = swerveModuleStates.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

  //optimize refrences state to eliminate rotation more than 90 degrees
  SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
       new Rotation2d(turningEncoder.getPosition()));

  //set drive and turning sparks to their setpoints
  //drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  /*driveVelocity.Velocity = optimizedDesiredState.speedMetersPerSecond * ModuleConstants.WHEEL_CIRCUMFRENCE_METERS;
            driveVelocity.FeedForward = ModuleConstants.DRIVE_FF;*/
            driveKraken.setControl(new VelocityVoltage(optimizedDesiredState.speedMetersPerSecond).withFeedForward(feedforward.calculate(optimizedDesiredState.speedMetersPerSecond)));
            SmartDashboard.putNumber("Velocity", Math.abs(optimizedDesiredState.speedMetersPerSecond));
    
  turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

  desiredState = optimizedDesiredState;
}

//zeros all swerve modules encoders 
public void resetEncoders() {
  driveKraken.setPosition(0);
}

public double getWheelVelocity()
{
  return driveKraken.getVelocity().getValue();
}

public double getDesiredVelocity()
{
  return desiredState.speedMetersPerSecond;
}

public double getRotations()
{
  return driveKraken.getVelocity().getValue();
}
}
