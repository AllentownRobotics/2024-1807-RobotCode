// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int operatorControllerID = 0;
  }


  public static class LightConstants {
    public static final int brightness = 255;
    public static final double speed = 0.2;
    public static final int ledNumber = 60;
    public static final int[] redbirdRed = {255, 0, 0};

    public static final StrobeAnimation ringColor = new StrobeAnimation(255, 172, 28, brightness, speed, ledNumber);
  }

  public static class CollectorConstants {

    public static final int leftMotorID = 0;
    public static final int rightMotorID = 1;
    public static final double collectSpeed = 1.0;
    public static final double collectSeconds = 0.5;
  }
}
