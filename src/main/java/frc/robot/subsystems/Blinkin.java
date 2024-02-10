// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase {
  PWMSparkMax blinkin;
  Collector collector;

  /** Creates a new Blinkin. */
  public Blinkin() {
    blinkin = new PWMSparkMax(BlinkinConstants.blinkinID);
  }

  public void BlinkinSet(double color) {
    blinkin.set(color);
  }

  public void BlinkinCustom() {
    blinkin.set(BlinkinConstants.blinkinWaves);
    //blinkin.set(BlinkinConstants.blinkinWhite);
    //blinkin.set(BlinkinConstants.blinkinRed);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
