// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

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
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .apply(MOTOR_CONFIG_LEFT)
        .follow(CAN_ID.LEFT_SHOOTER_MOTOR, true);

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("1:1")
    );

    public static final EncoderConfig ENCODER_CONFIG = MOTOR_CONFIG_LEFT.encoder
      .positionConversionFactor(GEARING.getRotorToMechanismRatio())
      .velocityConversionFactor(GEARING.getRotorToMechanismRatio() / 60.0);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);
  }

  // The hood's max RPS is ~0.37
  // The encoder shaft max RPS is ~3.87 or ~232 RPM
  public static final class HOOD {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(0.25)
      .smartCurrentLimit(20, 10)
      .voltageCompensation(12); //

    public static final AngularVelocity NEO_550_MAX_VEL = RPM.of(11000);

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("5:1", "9:1", "1:1", "166:20")
    );

    public static final AngularVelocity MAX_POSSIBLE_VELOCITY =
      NEO_550_MAX_VEL.times(GEARING.getRotorToMechanismRatio());

    public static final EncoderConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.encoder
      .positionConversionFactor(GEARING.getRotorToMechanismRatio())
      .velocityConversionFactor(GEARING.getRotorToMechanismRatio() / 60);

    public static final MechanismGearing ENCODER_GEARING = new MechanismGearing(
      GearBox.fromStages("166:20")
    );

    public static final AbsoluteEncoderConfig ABSOLUTE_ENCODER_CONFIG =
      MOTOR_CONFIG.absoluteEncoder
        .positionConversionFactor(ENCODER_GEARING.getRotorToMechanismRatio())
        .velocityConversionFactor(ENCODER_GEARING.getRotorToMechanismRatio());

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);
  }

  public static final class INTAKE_PIVOT {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(0.25)
      .smartCurrentLimit(40, 40)
      .voltageCompensation(12); //

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG =
      MOTOR_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    public static final AngularVelocity NEO_MAX_VEL = RPM.of(5676);

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("12:52", "16:54") //TODO: Ensure accurate IntakePivot Gearing
    );

    public static final AngularVelocity MAX_POSSIBLE_VELOCITY =
      NEO_MAX_VEL.times(GEARING.getRotorToMechanismRatio());

    public static final EncoderConfig ENCODER_CONFIG = MOTOR_CONFIG.encoder
      .positionConversionFactor(GEARING.getRotorToMechanismRatio())
      .velocityConversionFactor(GEARING.getRotorToMechanismRatio() / 60.0);

    public static final MechanismGearing ENCODER_GEARING = new MechanismGearing(
      GearBox.fromStages("1:1")
    );

    public static final AbsoluteEncoderConfig ABSOLUTE_ENCODER_CONFIG =
      MOTOR_CONFIG.absoluteEncoder
        .positionConversionFactor(ENCODER_GEARING.getRotorToMechanismRatio())
        .velocityConversionFactor(
          ENCODER_GEARING.getRotorToMechanismRatio() / 60.0
        );

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(40);
  }
}
