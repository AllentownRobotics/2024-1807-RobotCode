// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  
  CANdle candle = new CANdle(10);

  public LightAnim currentAnim = LightAnim.defaultAnim;

    public Lights() {
    CANdleConfiguration candleConfig = new CANdleConfiguration();
    candleConfig.stripType = LEDStripType.RGB;
    candleConfig.brightnessScalar = 1.0;
    candle.configAllSettings(candleConfig);

    setAnimation(LightAnim.defaultAnim);
  }

  public Lights setAnimation(LightAnim defaultanim) {
    currentAnim = defaultanim;
    defaultanim.run(candle);

    if (defaultanim == LightAnim.nullAnim) {
      candle.clearAnimation(0);
      candle.clearAnimation(1);
      candle.clearAnimation(2);
      candle.clearAnimation(3);
      candle.clearAnimation(4);
    }
    return this;
  }

  public void transitionCycle() {
    setAnimation(LightAnim.nullAnim);

    //add the autonomous commands later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
