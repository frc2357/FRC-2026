package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class Constants {

  public static final class CONTROLLER {

    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVER_CONTROLLER_DEADBAND = 0.025;
    public static final double DRIVER_RUMBLE_INTENSITY = .5;
    public static final double CODRIVER_RUMBLE_INTENSITY = .5;
    public static final double DRIVER_RUMBLE_SECONDS = 2;
    public static final double CODRIVER_RUMBLE_SECONDS = 2;
    public static final double JOYSTICK_RAMP_EXPONENT = 1;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED =
      TunerConstants.kSpeedAt12Volts;

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Units.Percent.of(
      50
    );
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);
  }

  public static final class PHOTON_VISION {

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
      "**************LOST CONNECTION WITH ORANGE PI";
    public static final String CONNECTION_REGAINED_MESSAGE =
      "CONNECTION REGAINED WITH ORANGE PI*********";

    public static final double MAX_ANGLE = 45;

    public static final class BACK_RIGHT_CAM {

      public static final String NAME = "backRight";
      // real transform
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-4.624),
        Units.Inches.of(7.799),
        Units.Inches.of(22.055),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(10),
          Units.Degrees.of(180)
        )
      );
    }

    public static final class BACK_LEFT_CAM {

      public static final String NAME = "backLeft";
      // true transform
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-6.516),
        Units.Inches.of(-5.028),
        Units.Inches.of(21.137),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(10),
          Units.Degrees.of(180)
        )
      );
    }
  }

  public static final class CHOREO {

    public static final PIDController X_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(
      8,
      0,
      0
    );

    public static final AutoFactory AUTO_FACTORY = new AutoFactory(
      Robot.swerve::getFieldRelativePose2d,
      Robot.swerve::setFieldRelativePose2d,
      Robot.swerve::followChoreoPath,
      true,
      Robot.swerve
    );
  }

  public static final class CAN_ID {

    public static final int SPINDEXER_MOTOR = 23;

    public static final int LEFT_INTAKE_MOTOR = 24;
    public static final int RIGHT_INTAKE_MOTOR = 25;

    public static final int KICKER_MOTOR = 26;

    public static final int OUTAKE_MOTOR = 27;

    public static final int HOOD_MOTOR = 28;

    public static final int OUTTAKE_MOTOR = 31;

    public static final int LEFT_SHOOTER_MOTOR = 29;
    public static final int RIGHT_SHOOTER_MOTOR = 30;
  }

  public static final class LED {

    public static final int kPort = 7;
    public static final int kLength = 86;
    //Change later
  }

  public static final class SPINDEXER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(40, 40)
      .openLoopRampRate(0.25); // TODO: double check these values

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);
  }

  public static final class INTAKE {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(75);

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(30, 30)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .apply(LEFT_MOTOR_CONFIG)
        .follow(CAN_ID.LEFT_INTAKE_MOTOR, true);
  }

  public static final class OUTTAKE {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(100);

    public static final SparkBaseConfig OUTTAKE_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(30, 30)
      .openLoopRampRate(0.5)
      .voltageCompensation(12);
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

    public static final double LEFT_MOTOR_P = 0;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_VEL_F = 0;
    public static final double LEFT_MOTOR_ARB_F = 0;
    public static final double MAX_VEL = 0;
    public static final double MAX_ACCEL = 0; //TODO: find actual values

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final double RPM_TOLERANCE = 100; //

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_VEL_F)
        .outputRange(-1, 1);
  }

  public static final class HOOD {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(20, 20)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);
  }
}
