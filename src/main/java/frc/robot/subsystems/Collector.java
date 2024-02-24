// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
    private CANSparkMax frontCollectorMotor;
    private CANSparkMax backCollectorMotor;

  /** Creates a new Collector. */
  public Collector() {
    frontCollectorMotor = new CANSparkMax(CollectorConstants.leftMotorID, MotorType.kBrushless);
    backCollectorMotor = new CANSparkMax(CollectorConstants.rightMotorID, MotorType.kBrushless);

    frontCollectorMotor.setIdleMode(IdleMode.kBrake);
    backCollectorMotor.setIdleMode(IdleMode.kBrake);

  }

  public void collect(double collectSpeed) {
    frontCollectorMotor.set(collectSpeed);
    backCollectorMotor.set(collectSpeed);
    frontCollectorMotor.get();
    backCollectorMotor.get();

  }

  public boolean collectingLight() {
    if (frontCollectorMotor.get() >= CollectorConstants.collectSpeed) {
    return true;
    } else if (backCollectorMotor.get() >= CollectorConstants.collectSpeed) {
    return true;
    } else {
    return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
