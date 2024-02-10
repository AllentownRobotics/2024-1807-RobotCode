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

  public static class BlinkinConstants { //color values found from blinkin docs
    
    public static final int blinkinID = 3;
    public static final double blinkinWaves = 0.53;
    public static final double blinkinOrange = -0.59;
    public static final double blinkinDarkRed = 0.59;
    public static final double blinkinWhite = 0.93;
    public static final double blinkinGold = 0.67;
    public static final double blinkinViolet = 0.91;
    public static final double blinkinWavesLava = -0.39;
    
  }

  public static class CollectorConstants {

    public static final int leftMotorID = 0;
    public static final int rightMotorID = 1;
    public static final double collectSpeed = 1.0;
    public static final double collectSeconds = 0.5;
  }

  public static class IndexerConstants {

    public static final int indexMotorID = 2;
    public static final double indexSpeed = 1.0;
  }
}
