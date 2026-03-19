package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Constants {

  public static final class ROBOT {

    public static final boolean PERFORMANCE_MODE = true;

    public static final TelemetryVerbosity MECHANISM_VERBOSITY =
      Constants.ROBOT.PERFORMANCE_MODE
        ? TelemetryVerbosity.LOW
        : TelemetryVerbosity.HIGH;
  }

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

    public static final Time TIME_TO_COAST = Seconds.of(3);

    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED =
      TunerConstants.kSpeedAt12Volts;

    public static final AngularVelocity MAX_DRIVE_AT_ANGLE_ANGULAR_RATE =
      RadiansPerSecond.of(2);

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Percent.of(100);
    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final double HEADING_CONTROLLER_P = 4.5;
    public static final double HEADING_CONTROLLER_I = 0;
    public static final double HEADING_CONTROLLER_D = 0;

    public static final Dimensionless INTAKE_TRANSLATION_MODIFIER = Percent.of(
      55
    );
    public static final Dimensionless INTAKE_ROTATION_MODIFIER = Percent.of(75);

    public static final Dimensionless SCORE_TRANSLATION_MODIFIER = Percent.of(
      30
    );
    public static final Dimensionless SCORE_ROTATION_MODIFIER = Percent.of(75);

    public static final Angle TELEOP_SHOOT_DRIVE_ANGLE_TOLERANCE = Degrees.of(
      15
    );
  }

  public static final class PHOTON_VISION {

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
      "**************LOST CONNECTION WITH ORANGE PI";
    public static final String CONNECTION_REGAINED_MESSAGE =
      "CONNECTION REGAINED WITH ORANGE PI*********";

    public static final double MAX_ANGLE = 45;

    public static final int NAIVE_APRIL_TAG_PIPELINE = 1;

    public static final int MULTI_TAG_PIPELINE = 0;

    public static final long NAIVE_APRIL_TAG_TARGET_TIMEOUT = 50;

    // TODO: These values could be fine tuned for the robot
    public static final class FILTER_PARAM {

      public static final LinearVelocity MAX_ROBOT_TRANSLATION =
        MetersPerSecond.of(2);
      public static final AngularVelocity MAX_ROBOT_ROTATION =
        RadiansPerSecond.of(3);
      public static final Distance MAX_DISTANCE_FROM_ROBOT = Meters.of(0.5);
    }

    public static final class KELPY_BACK_LEFT_CAM {

      public static final String NAME = "backLeft";

      // Camera flipped
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Inches.of(-9.516),
        Inches.of(-5.028),
        Inches.of(21.137),
        new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(180))
      );

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      // These are the default values from PhotonVision docs. They can be tuned per camera
      // by placing the robot at several points, recording the pose estimate and recording
      // the standard deviations
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
        4,
        4,
        Double.MAX_VALUE
      );
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
        0.5,
        0.5,
        Double.MAX_VALUE
      );
    }

    public static final class SHOOTER_CAM {

      public static final String NAME = "shooter";

      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Inches.of(-9.004),
        Inches.of(12.554),
        Inches.of(15.993),
        new Rotation3d(Degrees.of(10), Degrees.of(0), Degrees.of(90))
      );

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      // These are the default values from PhotonVision docs. They can be tuned per camera
      // by placing the robot at several points, recording the pose estimate and recording
      // the standard deviations
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
        4,
        4,
        Double.MAX_VALUE
      );
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
        0.5,
        0.5,
        Double.MAX_VALUE
      );
    }

    public static final class BACK_LEFT_CAM {

      public static final String NAME = "backLeft";

      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Inches.of(-14.116980),
        Inches.of(-11.594771),
        Inches.of(8.689800),
        new Rotation3d(
          Degrees.of(6.964),
          Degrees.of(7.177),
          Degrees.of(135.864)
        )
      );

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      // These are the default values from PhotonVision docs. They can be tuned per camera
      // by placing the robot at several points, recording the pose estimate and recording
      // the standard deviations
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
        4,
        4,
        Double.MAX_VALUE
      );
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
        0.5,
        0.5,
        Double.MAX_VALUE
      );
    }

    public static final class BACK_RIGHT_CAM {

      public static final String NAME = "backRight";

      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Inches.of(-14.116980),
        Inches.of(-11.594771),
        Inches.of(8.689800),
        new Rotation3d(
          Degrees.of(-6.964),
          Degrees.of(7.177),
          Degrees.of(225.864)
        )
      );

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      // These are the default values from PhotonVision docs. They can be tuned per camera
      // by placing the robot at several points, recording the pose estimate and recording
      // the standard deviations
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
        4,
        4,
        Double.MAX_VALUE
      );
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
        0.5,
        0.5,
        Double.MAX_VALUE
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

    public static final int FLOOR_MOTOR = 23; //TODO: make sure that the old spindexter motor is the same for the floor (or just change the CAN ID)
    public static final int TUNNEL_MOTOR = 34;

    public static final int LEFT_INTAKE_MOTOR = 24;
    public static final int RIGHT_INTAKE_MOTOR = 25;
    public static final int INTAKE_PIVOT_MOTOR = 33;

    public static final int FEEDER_MOTOR = 32;
    public static final int KICKER_MOTOR = 35;

    public static final int HOOD_MOTOR = 28;

    public static final int LEFT_SHOOTER_MOTOR = 29;
    public static final int RIGHT_SHOOTER_MOTOR = 30;
  }

  public static final class FLOOR {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .smartCurrentLimit(40, 20)
      .openLoopRampRate(0.25)
      .voltageCompensation(10);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final Dimensionless FLOOR_SPEED = Percent.of(80);
  }

  public static final class TUNNEL {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .openLoopRampRate(0.25)
      .smartCurrentLimit(30, 20)
      .voltageCompensation(12);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);
    public static final Dimensionless TUNNEL_SPEED = Percent.of(100);
    public static final Dimensionless REVERSE_TUNNEL_SPEED = Percent.of(-100);
  }

  public static final class INTAKE_RUNNER {

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(75);

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .smartCurrentLimit(40, 20)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .apply(LEFT_MOTOR_CONFIG)
        .follow(CAN_ID.LEFT_INTAKE_MOTOR, true);

    public static final Dimensionless TELEOP_INTAKING_SPEED = Percent.of(70);
    public static final Dimensionless INTAKE_JIGGLING_SPEED = Percent.of(10);
  }

  public static final class INTAKE_PIVOT {

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("12:52", "16:54")
    );

    // Diameter of the arm.
    public static final Distance LENGTH = Inches.of(12.5);
    // Mass of the arm.
    public static final Mass MASS = Pounds.of(7);

    // Telemetry name and verbosity for the arm.
    public static final String MECHANISM_NETWORK_KEY = "IntakePivotMech";
    public static final String MOTOR_NETWORK_KEY = "IntakePivotMotor";

    public static final Angle SIM_LOWER_ANGLE = Degrees.of(0);
    public static final Angle SIM_UPPER_ANGLE = Degrees.of(123.3);
    public static final Angle SIM_STARTING_POSITION = Degrees.of(20);

    public static final Current STALL_LIMIT = Amps.of(40);

    public static final SparkBaseConfig INTAKE_PIVOT_BASE_CONFIG =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) STALL_LIMIT.in(Amps), 40)
        .voltageCompensation(12);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(50);

    public static final Dimensionless HOLD_DOWN_SPEED = Percent.of(-5);

    public static final Dimensionless RETRACT_SPEED = Percent.of(30);

    public static final Dimensionless DEPLOY_SPEED = Percent.of(-30);

    public static final Dimensionless JIGGLE_UP_SPEED = Percent.of(20);
    public static final Dimensionless JIGGLE_DOWN_SPEED = Percent.of(-10);

    public static final Time JIGGLE_UP_TIME = Seconds.of(0.3);

    public static final Current AMP_STALL_THRESHOLD = Amps.of(35);
    public static final Time TIME_TO_STALL = Seconds.of(0.1);

    // When intake is zeroed at the upper hard stops, fully deployed is about -160 rotations
    public static final Angle INTAKE_DEPLOYED_ENCODER_ROTATIONS = Rotations.of(
      -125
    );

    // Maximum amount of time we can stall the intake against the hardstop without ripping it off
    public static final Time INTAKE_MAXIMUM_STALL_TIME = Seconds.of(3);
  }

  public static final class FEEDER {

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final Current STALL_LIMIT = Amps.of(40);

    public static final boolean INVERTED = true;

    public static final SparkBaseConfig FEEDER_BASE_CONFIG =
      new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .inverted(INVERTED)
        .smartCurrentLimit((int) STALL_LIMIT.in(Amps), 10)
        .openLoopRampRate(0.25)
        .voltageCompensation(12);

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("1:1")
    );

    // Diameter of the flywheel.
    public static final Distance DIAMETER = Inches.of(2);
    // Mass of the flywheel.
    public static final Mass MASS = Pounds.of(1);
    // Maximum speed of the shooter.

    public static final String MECHANISM_NETWORK_KEY = "FeederMech";
    public static final String MOTOR_NETWORK_KEY = "FeederMotor";

    // TODO: PID, Feedforward, max angular acceleration still need tuned for mechanism
    public static final double P = 0.005;
    public static final double I = 0;
    public static final double D = 0;
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(
      5676
    ).times(1);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
      RotationsPerSecondPerSecond.of(150);

    public static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(0.15, 0.125, 0.0);

    public static final Dimensionless FEED_SPEED_PERCENT = Percent.of(100);
    public static final Dimensionless REVERSE_FEED_SPEED_PERCENT = Percent.of(
      -100
    );

    public static final AngularVelocity FEED_SPEED = RotationsPerSecond.of(77);
  }

  public static final class KICKER {

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final SparkBaseConfig KICKER_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .smartCurrentLimit(15, 10)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);

    public static final Dimensionless KICK_SPEED = Percent.of(75);
    public static final Dimensionless REVERSE_KICK_SPEED = Percent.of(-75);
  }

  public static final class SHOOTER {

    public static final double STEP_AXIS_STEP = 0.1;

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("1:1")
    );

    // Diameter of the flywheel.
    public static final Distance DIAMETER = Inches.of(4);
    // Mass of the flywheel.
    public static final Mass MASS = Pounds.of(2.717);
    // Maximum speed of the shooter.

    public static final String MECHANISM_NETWORK_KEY = "ShooterMech";
    public static final String MOTOR_NETWORK_KEY = "ShooterMotor";

    public static final Current STALL_LIMIT = Amps.of(40);

    public static final SparkBaseConfig SHOOTER_BASE_CONFIG =
      new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) STALL_LIMIT.in(Amps), 40)
        .voltageCompensation(12);

    // TODO: PID, Feedforward, max angular acceleration still need tuned for mechanism
    public static final double P = 0.005;
    public static final double I = 0;
    public static final double D = 0;
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(5676);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
      RotationsPerSecondPerSecond.of(150);

    public static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(0.12, 0.125, 0.01);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final AngularVelocity INITIAL_SCORE_TOLERANCE =
      RotationsPerSecond.of(1);
    public static final Dimensionless CONTINUOUS_SCORE_TOLERANCE = Percent.of(
      10
    );
    public static final Time STABLE_VELOCITY = Seconds.of(0.1);

    public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(
      Inches.of(-6.781),
      Inches.of(-2.833),
      new Rotation2d(Degrees.of(90))
    );

    public static final class SETPOINTS {

      public static final AngularVelocity HUB_SHOT = RotationsPerSecond.of(45);

      public static final AngularVelocity TRENCH_SHOT = RotationsPerSecond.of(
        54
      );

      // Farthest corner possible
      public static final AngularVelocity OUTPOST_SHOT = RotationsPerSecond.of(
        64
      );
    }
  }

  public static final class HOOD {

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("5:1", "9:1", "1:1", "166:20")
    );

    public static final MechanismGearing ENCODER_GEARING = new MechanismGearing(
      GearBox.fromStages("166:20")
    );

    // This is the number that should be copied from the rev hardware client when
    // pressing the "zero encoder" button
    public static final Angle PHYSICAL_ZERO_OFFSET = Rotations.of(0.31635416);

    // Fabricated offset to prevent wrapping
    public static final Angle FABRICATED_ADJUSTMENT = Degrees.of(1).times(
      ENCODER_GEARING.getMechanismToRotorRatio()
    );
    public static final Angle ADJUSTED_ZERO_OFFSET = PHYSICAL_ZERO_OFFSET.minus(
      FABRICATED_ADJUSTMENT
    );

    public static final Angle LOWER_ANGLE_LIMIT = Degrees.of(0.9);
    public static final Angle UPPER_ANGLE_LIMIT = Degrees.of(34);
    public static final Angle SIM_STARTING_POSITION = Degrees.zero();

    // Mass of the flywheel.
    // Telemetry name and verbosity for the arm.
    public static final String MECHANISM_NETWORK_KEY = "HoodMech";
    public static final String MOTOR_NETWORK_KEY = "HoodMotor";

    public static final Current STALL_LIMIT = Amps.of(10);

    public static final SparkBaseConfig HOOD_BASE_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit((int) STALL_LIMIT.in(Amps), 10)
      .voltageCompensation(12);

    public static final SignalsConfig SIGNAL_CONFIG = HOOD_BASE_CONFIG.signals
      .absoluteEncoderPositionPeriodMs(20)
      .absoluteEncoderVelocityPeriodMs(20);

    public static final double P = 50;
    public static final double I = 0;
    public static final double D = 0;

    public static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(0.1, 0.0, 0.0);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(30);
    public static final Distance LENGTH = Inches.of(8);
    public static final Mass MASS = Pounds.of(1.365);

    public static final class SETPOINTS {

      public static final Angle HOME = Degrees.of(1);

      public static final Angle HUB_SHOT = Degrees.of(1);

      public static final Angle TRENCH_SHOT = Degrees.of(10.5);

      // Farthest corner possible
      public static final Angle OUTPOST_SHOT = Degrees.of(18);
    }

    public static final Angle PASSING_STATIC_ANGLE = Degrees.of(18); // TODO: Tune
  }

  public class SCORING {

    /**
     * The latency compensation to account for the time a ball is feeding, in the shooter,
     * or signals being sent to the motors for shoot on the fly algorithm
     *
     * I do not understand why the best value for this is zero
     *  */
    public static final Time SOTF_LATENCY_COMPENSATION = Seconds.of(0);

    public static final String IS_SOTF_KEY = "Enable SOTF";

    public static final int SOTF_CONVERGE_ITERATIONS = 20;
    public static final int DRIVE_ANGLE_CONVERGE_ITERATIONS = 2;

    public static final Time TIME_TO_WARN_FOR_ACTIVE_HUB = Seconds.of(10);

    public static final Rectangle2d[] NO_SHOOT_ZONES = new Rectangle2d[] {
      // This rect represents a 47in x 47in square centered on the neutral zone side of the hub.
      // Basically the hub if it were translated 47in towards the neutral zone.
      new Rectangle2d(
        FieldConstants.Hub.farRightCorner,
        FieldConstants.Hub.farRightCorner.plus(
          new Translation2d(FieldConstants.Hub.width, FieldConstants.Hub.width)
        )
      ),
    };
  }

  public class AUTO {

    public static final AngularVelocity AUTO_PASSING_SHOOTER_VELOCITY =
      RotationsPerSecond.of(40);
    public static final Angle AUTO_PASSING_HOOD_ANGLE = Degrees.of(15);
  }

  public class SHIFT {

    public static final Time AUTO_LENGTH = Seconds.of(20);
    public static final Time TELEOP_LENGTH = Seconds.of(140);

    // Times relative to teleop (start of teleop = 0)
    public static final Time[] TELEOP_SHIFT_START_TIMES = {
      Seconds.of(0),
      Seconds.of(10.0),
      Seconds.of(35.0),
      Seconds.of(60.0),
      Seconds.of(85.0),
      Seconds.of(110.0),
    };
    public static final Time[] TELEOP_SHIFT_END_TIMES = {
      Seconds.of(10.0),
      Seconds.of(35.0),
      Seconds.of(60.0),
      Seconds.of(85.0),
      Seconds.of(110.0),
      Seconds.of(140.0),
    };

    // Order of when hub is active when we lose auto
    public static final boolean[] AUTO_LOSE_SCHEDULE = {
      true,
      true,
      false,
      true,
      false,
      true,
    };
    // Order of when hub is active when we win auto
    public static final boolean[] AUTO_WIN_SCHEDULE = {
      true,
      false,
      true,
      false,
      true,
      true,
    };
  }

  public class FieldConstants {

    public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // AprilTag related constants
    public static final int aprilTagCount = FIELD_LAYOUT.getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    // Field dimensions
    public static final double fieldLength = FIELD_LAYOUT.getFieldLength();
    public static final double fieldWidth = FIELD_LAYOUT.getFieldWidth();

    /** Hub related constants */
    public static class Hub {

      // Dimensions
      public static final double width = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth = Units.inchesToMeters(41.7);
      public static final double innerHeight = Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint = new Translation2d(
        FIELD_LAYOUT.getTagPose(26).get().getX() + width / 2.0,
        fieldWidth / 2.0
      );
      public static final Translation3d topCenterPoint = new Translation3d(
        centerPoint.getX(),
        centerPoint.getY(),
        height
      );
      public static final Translation3d innerCenterPoint = new Translation3d(
        centerPoint.getX(),
        centerPoint.getY(),
        innerHeight
      );

      public static final Translation2d nearLeftCorner = new Translation2d(
        topCenterPoint.getX() - width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
      public static final Translation2d nearRightCorner = new Translation2d(
        topCenterPoint.getX() - width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
      public static final Translation2d farLeftCorner = new Translation2d(
        topCenterPoint.getX() + width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
      public static final Translation2d farRightCorner = new Translation2d(
        topCenterPoint.getX() + width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );

      // Relevant reference points on the opposite side
      public static final Translation3d oppTopCenterPoint = new Translation3d(
        FIELD_LAYOUT.getTagPose(4).get().getX() + width / 2.0,
        fieldWidth / 2.0,
        height
      );
      public static final Translation2d oppNearLeftCorner = new Translation2d(
        oppTopCenterPoint.getX() - width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
      public static final Translation2d oppNearRightCorner = new Translation2d(
        oppTopCenterPoint.getX() - width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );
      public static final Translation2d oppFarLeftCorner = new Translation2d(
        oppTopCenterPoint.getX() + width / 2.0,
        fieldWidth / 2.0 + width / 2.0
      );
      public static final Translation2d oppFarRightCorner = new Translation2d(
        oppTopCenterPoint.getX() + width / 2.0,
        fieldWidth / 2.0 - width / 2.0
      );

      // Hub faces
      public static final Pose2d nearFace = FIELD_LAYOUT.getTagPose(26)
        .get()
        .toPose2d();
      public static final Pose2d farFace = FIELD_LAYOUT.getTagPose(20)
        .get()
        .toPose2d();
      public static final Pose2d rightFace = FIELD_LAYOUT.getTagPose(18)
        .get()
        .toPose2d();
      public static final Pose2d leftFace = FIELD_LAYOUT.getTagPose(21)
        .get()
        .toPose2d();
    }

    public static class LeftBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      public static final Translation2d centerPoint = new Translation2d(
        Hub.centerPoint.getX(),
        Hub.centerPoint.getY() + Hub.width / 2.0 + width / 2.0
      );

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner = new Translation2d(
        LinesVertical.hubCenter - width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner = new Translation2d(
        LinesVertical.hubCenter + width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner = new Translation2d(
        LinesVertical.hubCenter - width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d oppNearRightCorner =
        Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner = new Translation2d(
        LinesVertical.hubCenter + width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d oppFarRightCorner =
        Hub.oppFarLeftCorner;
    }

    public static class RightBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      public static final Translation2d centerPoint = new Translation2d(
        Hub.centerPoint.getX(),
        Hub.centerPoint.getY() - Hub.width / 2.0 - width / 2.0
      );

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner = new Translation2d(
        LinesVertical.hubCenter + width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner = new Translation2d(
        LinesVertical.hubCenter - width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner = new Translation2d(
        LinesVertical.hubCenter + width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d oppNearRightCorner =
        Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner = new Translation2d(
        LinesVertical.hubCenter - width / 2,
        Units.inchesToMeters(255)
      );
      public static final Translation2d oppFarRightCorner =
        Hub.oppFarLeftCorner;
    }

    public static class LinesVertical {

      public static final double center = fieldLength / 2.0;
      public static final double starting = FIELD_LAYOUT.getTagPose(26)
        .get()
        .getX();
      public static final double allianceZone = starting;
      public static final double hubCenter =
        FIELD_LAYOUT.getTagPose(26).get().getX() + Hub.width / 2.0;
      public static final double neutralZoneNear =
        center - Units.inchesToMeters(120);
      public static final double neutralZoneFar =
        center + Units.inchesToMeters(120);
      public static final double oppHubCenter =
        FIELD_LAYOUT.getTagPose(4).get().getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = FIELD_LAYOUT.getTagPose(10)
        .get()
        .getX();
    }
  }
}
