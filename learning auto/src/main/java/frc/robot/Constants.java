// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;

/*
The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
constants. This class should not be used for any other purpose. All constants should be declared
globally (i.e. public static). Do not put anything functional in this class.

It is advised to statically import this class (or one of its inner classes) wherever the
constants are needed, to reduce verbosity.
*/

public final class Constants
{

  public static final int kDriverControllerPort = 0;

  public static class kDrivetrain
  {

    public static final int kCurrentLimit = 80;
    public static final Sendable rampRate = null;

    public static class kMotor
    {

      public static final int kIdLeft1 = 20;
      public static final int kIdLeft2 = 21;
      public static final int kIdLeft3 = 22;

      public static final int kIdRight1 = 23;
      public static final int kIdRight2 = 24;
      public static final int kIdRight3 = 25;

    }

    public static class kWheel
    {
      public static final double kWheelDiam = 0.1016; /* diameter exactly in meters; original value is 4 inches */
      public static final double kWheelCircm = Math.PI * kWheelDiam;
    }

    public static class kCANCoder
    {

      public static final int kIdEncLeft = 30;
      public static final int kIdEncRight = 29;

      public static final int kCountsPerRev = 4096;
      public static final double kSensorCoeff = kDrivetrain.kWheel.kWheelCircm / kCountsPerRev;
      public static final String kEncUnit = "m";

    }
  }
}