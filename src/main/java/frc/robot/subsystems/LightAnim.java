// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants.LightConstants;

public interface LightAnim {
  void run(CANdle candle);

    public static final LightAnim nullAnim = (candle) -> candle.setLEDs(0, 0, 0);
    public static final LightAnim defaultAnim = (candle) -> candle.setLEDs(255, 0, 0);
    public static final LightAnim requestRing = (candle) -> candle.animate(LightConstants.ringColor, 2);
}
