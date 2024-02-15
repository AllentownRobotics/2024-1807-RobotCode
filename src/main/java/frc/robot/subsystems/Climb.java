// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private CANSparkMax climbLeftMotor;
  private CANSparkMax climbRightMotor;
  private double currentAverage;
  private double currentSnapshot;
  private CANSparkMax firstContactedMotor;
  private CANSparkMax secondContactedMotor;
  private Pigeon2 pigeon;
  private double tilt;
  private PIDController pidController;
  private double speed;

  /** Creates a new Climb. */
  public Climb() {

    climbLeftMotor = new CANSparkMax(ClimbConstants.climbLeftMotorID, MotorType.kBrushless);
    climbRightMotor = new CANSparkMax(ClimbConstants.climbRightMotorID, MotorType.kBrushless);

    pigeon = new Pigeon2(ClimbConstants.climbPigeonID);

    climbLeftMotor.restoreFactoryDefaults();
    climbRightMotor.restoreFactoryDefaults();

    climbLeftMotor.setIdleMode(IdleMode.kBrake);
    climbRightMotor.setIdleMode(IdleMode.kBrake);

    climbLeftMotor.setSmartCurrentLimit(ClimbConstants.climbSmartCurrentLimit);
    climbRightMotor.setSmartCurrentLimit(ClimbConstants.climbSmartCurrentLimit);

    climbLeftMotor.burnFlash();
    climbRightMotor.burnFlash();

    pidController = new PIDController
    (ClimbConstants.climbPID_P, ClimbConstants.climbPID_I, ClimbConstants.climb_PID_D);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // constantly calculates pid based on pigeon tilt
    speed = pidController.calculate(tilt);

    SmartDashboard.putNumber("Left Motor Current", climbLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Motor Current", climbRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("pid", speed);
  }

  // sets speed value of motors
  public void setLeftMotor(double power) {
    climbLeftMotor.set(power);
  }

  public void setRightMotor(double power) {
    climbRightMotor.set(power);
  }

  // establishes setpoint of pid
  public void setSetpoint() {
    pidController.setSetpoint(0);
  }
  
  // sets all the different ways of getting current data
  public double setCurrentSnapshot() {
    
    Commands.waitSeconds(ClimbConstants.climbCurrentPingDelayMilliseconds);
    
    return currentSnapshot = climbLeftMotor.getOutputCurrent();
  }
  
  public double setCurrentSampling() {
    Commands.waitSeconds(ClimbConstants.climbCurrentPingDelayMilliseconds);
    
    // gets a small sampling of current values
    double currentA = climbLeftMotor.getOutputCurrent();
    Commands.waitSeconds(ClimbConstants.climbCurrentPingDelayMilliseconds);
    
    double currentB = climbLeftMotor.getOutputCurrent();
    Commands.waitSeconds(ClimbConstants.climbCurrentPingDelayMilliseconds);
    
    double currentC = climbLeftMotor.getOutputCurrent();
    
    // averages current values;
    return currentAverage = (currentA + currentB + currentC) / 3;
  }
  
  // gets all the different types of current values
  public double getCurrentStatic() {
    return ClimbConstants.climbCurrentStaticValue;
  }

  public double getCurrentSnapshot() {
    return currentSnapshot;
  }

  public double getCurrentSampling() {
    return currentAverage;
  }

  // determines which motor contacted the chain first
  // and which motor contacted second
  public void setContactOrder(CANSparkMax firstContactedMotor, CANSparkMax secondContactedMotor) {
    this.firstContactedMotor = firstContactedMotor;
    this.secondContactedMotor = secondContactedMotor;
  }

  // gets which motor contacted in what order
  public CANSparkMax getFirstContact() {
    return firstContactedMotor;
  }

  public CANSparkMax getSecondContact() {
    return secondContactedMotor;
  }

  public boolean getIsFirstContactedPigeon() {

    // uses the tilt of the robot to see
    // which motor contacted first
    if (getTilt() < -ClimbConstants.climbMinTiltedDegrees) {

      // sets the order of which motor contacted when
      setContactOrder(getRightMotor(), getLeftMotor());
      
      // stops the first contacted motor and moves on
      getFirstContact().set(0);

      return true;
    } else if (getTilt() > ClimbConstants.climbMinTiltedDegrees) {

      // sets the order of which motor contacted when
      setContactOrder(getLeftMotor(), getRightMotor());

      // stops the first contacted motor and moves on
      setLeftMotor(0);

      return true;
    } else {

    return false;
    }
  }

  public boolean getIsSecondContactedPigeon() {

        // waits for the second motor to balance the robot and moves on
        if (Math.abs(getTilt()) < ClimbConstants.climbBalancedDegrees) {

          return true;
    
        } else {
    
          return false;
      }
  }

  public boolean getIsFirstContactedCurrent() {
  
    // checks for a current spike in either motor
    // afterward, it moves to the next step
    if (getCurrentLeft() - getCurrentSnapshot() > ClimbConstants.climbCurrentDifference) {

      // disables the contacted motor and
      // sets it as first contacted
      setLeftMotor(0);
      setContactOrder(getLeftMotor(), getRightMotor());

      return true;
    } else if (getCurrentRight() - getCurrentSnapshot() > ClimbConstants.climbCurrentDifference) {

      // disables the contacted motor and
      // sets it as first contacted
      setRightMotor(0);
      setContactOrder(getRightMotor(), getLeftMotor());

      return true;
    } else {
      return false;
    }
  }

  public boolean getIsSecondContactedCurrent() {
        // checks the second motor and waits for a current spike
        if (getSecondContact().getOutputCurrent() - getCurrentSnapshot() > 
        ClimbConstants.climbCurrentDifference) {
          
          // stops the second motor and moves on
          getSecondContact().set(0);
    
          return true;
        } else {
    
          return false;
        }
  }

  // gets the current of each of the motors
  public double getCurrentLeft() {
    return climbLeftMotor.getOutputCurrent();
  }

  public double getCurrentRight() {
    return climbRightMotor.getOutputCurrent();
  }

  // gets the motors themselves
  public CANSparkMax getLeftMotor() {
    return climbLeftMotor;
  }

  public CANSparkMax getRightMotor() {
    return climbRightMotor;
  }

  // gets the value of the pid
  public double getSpeed() {
    return pidController.calculate(tilt);
  }
  
  // gets the direct tilt of the pigeon in degrees
  public double getTilt() {
    return pigeon.getPitch().getValueAsDouble();
  }
}
