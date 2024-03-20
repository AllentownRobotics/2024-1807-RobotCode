// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private CANSparkMax frontCollectorMotor;
  private CANSparkFlex rearCollectorMotor;
  private DigitalInput collectorBeamBreak;

  /** Creates a new Collector. */
  public Collector() {
    frontCollectorMotor = new CANSparkMax(CollectorConstants.frontCollectorMotorID, MotorType.kBrushless);
    rearCollectorMotor = new CANSparkFlex(CollectorConstants.rearCollectorMotorID, MotorType.kBrushless);

    frontCollectorMotor.restoreFactoryDefaults();
    rearCollectorMotor.restoreFactoryDefaults();

    frontCollectorMotor.setIdleMode(IdleMode.kBrake);
    rearCollectorMotor.setIdleMode(IdleMode.kBrake);

    frontCollectorMotor.setInverted(true);
    rearCollectorMotor.follow(frontCollectorMotor);

    frontCollectorMotor.burnFlash();
    rearCollectorMotor.burnFlash();

    collectorBeamBreak = new DigitalInput(CollectorConstants.collectorBeamBreakPort);
  }

  public void collect(double collectSpeed)
  {
    frontCollectorMotor.set(collectSpeed);
    rearCollectorMotor.set(collectSpeed);
  }

  /**Returns state of the beam break for the ground collection 
   * @return True if beam break is broken, false otherwise
   */
  public boolean getCollectorBeamBreak()
  {
    return !collectorBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}