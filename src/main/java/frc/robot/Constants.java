// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (100) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.1, 0, 0.01);

    public static final double MAX_SPEED        = 0.5;
    public static final double MAX_ACCELERATION = 1;
  }
  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.22, 0, 0.0);
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME =  10; // seconds
    public static final double WHEEL_RADIUS =     4; // inches
    public static final double DRIVE_GEAR_RATIO = 6.75; // gear ratios
    public static final double ANGLE_GEAR_RATIO = 12.8; // gear ratios
  }
  public static final class Shooter{
    public final static int SHOOTER_MOTORONE_CAN = 14;
    public final static int SHOOTER_MOTORTWO_CAN = 15;
    public final static int SHOOTER_PIVOTONE_CAN = 16;
    public final static int SHOOTER_PIVOTTWO_CAN = 17;
    public final static int SHOOTER_PIVOTTHREE_CAN = 12;
    public final static double FALL_CANCEL_SPEED = 0.1;
    public final static int PIVOT_CURRENT_LIMIT = 10;
    public final static int PIVOT_CANCODER_ID = 21;
    //pid controller
    public final static double SHOOTER_KP = 0.001;
    public final static double SHOOTER_KI = 0.00;
    public final static double SHOOTER_KD = 0.00;
    public final static double SHOOTER_MAX_VELOCITY = 0.1;
    public final static double SHOOTER_MAX_ACCEL = 0.1;
    public final static ProfiledPIDController SHOOTER_PID_CONTROLLER = new ProfiledPIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, new Constraints(SHOOTER_MAX_VELOCITY, SHOOTER_MAX_ACCEL));
    //feedforward
    public final static double SHOOTER_KA = 0.08;
    public final static double SHOOTER_KV = 1.56;
    public final static double SHOOTER_KS = 0.00;
    public final static double SHOOTER_KG = 1.55;
    public final static ArmFeedforward SHOOTER_FF_CONTROLLER = new ArmFeedforward(SHOOTER_KS, SHOOTER_KG, SHOOTER_KV);

  }
  public static final class Intake
  {
    public static final int DISTSENSOR_ANALOG_ID = 4;
    public static final int INTAKE_CAN_ONE = 8;
    public static final int INTAKE_CAN_TWO = 19;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  }
  public static final class FieldConstants{
    public static final int SPEAKER_HEIGHT = 7 * 12;
    public static final double SPEAKER_X_BLUE = 0;
    public static final double SPEAKER_X_RED = 54;
    public static final double AMP_ANGLE = 45;
  }
  public static final class BTS{
    public static final int BTS_MOTOR_CAN = 10;
  }
  public static class OperatorConstants
  {
    // Controller Ports
    public static final int JOYSTICK_1_PORT = 0;
    public static final int JOYSTICK_2_PORT = 1;
    public static final int GUITAR_PORT = 2;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.001;
    public static final double LEFT_Y_DEADBAND = 0.001;
    public static final double RIGHT_X_DEADBAND = 0.001;
    public static final double TURN_CONSTANT = 0.075;
  }
}
