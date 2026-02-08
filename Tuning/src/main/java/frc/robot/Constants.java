// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CONTROLLER {

    public static final int DRIVER_CONTROLLER_PORT = 1;
  }

  public static final class CAN_ID {

    public static final int SPINDEXER_MOTOR = 23;

    public static final int LEFT_INTAKE_MOTOR = 24;
    public static final int RIGHT_INTAKE_MOTOR = 25;
    public static final int INTAKE_PIVOT_MOTOR = 33;

    public static final int KICKER_MOTOR = 26;
    public static final int FEEDER_MOTOR = 32;
    //Feeder motor can ID

    public static final int OUTAKE_MOTOR = 27;

    public static final int HOOD_MOTOR = 28;

    public static final int OUTTAKE_MOTOR = 31;

    public static final int LEFT_SHOOTER_MOTOR = 29;
    public static final int RIGHT_SHOOTER_MOTOR = 30;
  }

  public static final class SHOOTER {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .openLoopRampRate(0.25)
      .smartCurrentLimit(40, 40)
      .voltageCompensation(12); //

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .apply(MOTOR_CONFIG_LEFT)
        .follow(CAN_ID.LEFT_SHOOTER_MOTOR, true);

    public static final double LEFT_MOTOR_kP = 0;
    public static final double LEFT_MOTOR_kI = 0;
    public static final double LEFT_MOTOR_kD = 0;
    public static final double LEFT_MOTOR_kS = 0;
    public static final double LEFT_MOTOR_kV = 0;
    public static final double LEFT_MOTOR_kA = 0;
    public static final double MAX_VEL = 0;
    public static final double MAX_ACCEL = 0; //TODO: find actual values
    public static final double RPM_TOLERANCE = 100; //

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .outputRange(-1, 1)
        .pid(LEFT_MOTOR_kP, LEFT_MOTOR_kI, LEFT_MOTOR_kD);

    public static final FeedForwardConfig FEED_FORWARD_CONFIG =
      CLOSED_LOOP_CONFIG_LEFT.feedForward.sva(
        LEFT_MOTOR_kS,
        LEFT_MOTOR_kV,
        LEFT_MOTOR_kA
      );

    public static final MAXMotionConfig MAX_MOTION_CONFIG =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedProfileError(RPM_TOLERANCE)
        .maxAcceleration(MAX_ACCEL)
        .cruiseVelocity(MAX_VEL);
  }

  public static final class INTAKE_PIVOT {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(75);

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(30, 30)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kCosG = 0;
    public static final double MAX_VEL = 0;
    public static final double MAX_ACCEL = 0; //TODO: find actual values
    public static final double ANGULAR_TOLERANCE = 100; //

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG.closedLoop.outputRange(-1, 1).pid(kP, kI, kD);

    public static final FeedForwardConfig FEED_FORWARD_CONFIG =
      CLOSED_LOOP_CONFIG_LEFT.feedForward.sva(kS, kV, kA);

    public static final MAXMotionConfig MAX_MOTION_CONFIG =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedProfileError(ANGULAR_TOLERANCE)
        .maxAcceleration(MAX_ACCEL)
        .cruiseVelocity(MAX_VEL);
  }
}
