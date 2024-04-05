// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  private CANSparkMax frontCollectorMotor;
  private CANSparkFlex rearCollectorMotor;
  private DigitalInput rearCollectorBeamBreak;
  private DigitalInput frontCollectorBeamBreak;

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

    //frontCollectorMotor.setSmartCurrentLimit(50);
    //rearCollectorMotor.setSmartCurrentLimit(50);

    frontCollectorMotor.burnFlash();
    rearCollectorMotor.burnFlash();

    rearCollectorBeamBreak = new DigitalInput(CollectorConstants.rearCollectorBeamBreakPort);
    frontCollectorBeamBreak = new DigitalInput(CollectorConstants.frontCollectorBeamBreakPort);
  }

  public void collect(double collectSpeed)
  {
    frontCollectorMotor.set(collectSpeed);
    rearCollectorMotor.set(collectSpeed);
  }

  /**Returns state of the beam break for the ground collection 
   * @return True if beam break is broken, false otherwise
   */
  public boolean getRearCollectorBeamBreak()
  {
    return !rearCollectorBeamBreak.get();
  }

  public boolean getFrontCollectorBeamBreak()
  {
    return !frontCollectorBeamBreak.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}