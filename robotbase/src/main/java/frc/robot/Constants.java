package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import choreo.auto.AutoFactory;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
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

    public static final Time TIME_TO_COAST = Units.Seconds.of(3);

    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(1);
    public static final LinearVelocity MAX_SPEED =
      TunerConstants.kSpeedAt12Volts;

    public static final AngularVelocity MAX_DRIVE_AT_ANGLE_ANGULAR_RATE =
      RadiansPerSecond.of(2);

    public static final Dimensionless AXIS_MAX_ANGULAR_RATE = Units.Percent.of(
      50
    );
    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(50);

    public static final double HEADING_CONTROLLER_P = 4.5;
    public static final double HEADING_CONTROLLER_I = 0;
    public static final double HEADING_CONTROLLER_D = 0;
  }

  public static final class PHOTON_VISION {

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
      "**************LOST CONNECTION WITH ORANGE PI";
    public static final String CONNECTION_REGAINED_MESSAGE =
      "CONNECTION REGAINED WITH ORANGE PI*********";

    public static final double MAX_ANGLE = 45;

    public static final int NAIVE_APRIL_TAG_PIPELINE = 0;

    public static final int MULTI_TAG_PIPELINE = 1;

    public static final long NAIVE_APRIL_TAG_TARGET_TIMEOUT = 50;

    public static final class BACK_RIGHT_CAM {

      public static final String NAME = "backRight";
      // real transform
      // public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
      //   Units.Inches.of(-4.624),
      //   Units.Inches.of(7.799),
      //   Units.Inches.of(22.055),
      //   new Rotation3d(
      //     Units.Degrees.of(0),
      //     Units.Degrees.of(10),
      //     Units.Degrees.of(180)
      //   )
      // );

      // Transform of camera flipped
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-4.624),
        Units.Inches.of(7.299),
        Units.Inches.of(22.055),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(-10),
          Units.Degrees.of(180)
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

    public static final class BACK_LEFT_CAM {

      public static final String NAME = "backLeft";
      // true transform
      // public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
      //   Units.Inches.of(-6.516),
      //   Units.Inches.of(-5.028),
      //   Units.Inches.of(21.137),
      //   new Rotation3d(
      //     Units.Degrees.of(0),
      //     Units.Degrees.of(10),
      //     Units.Degrees.of(180)
      //   )
      // );

      // Camera flipped
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-9.516),
        Units.Inches.of(-5.028),
        Units.Inches.of(21.137),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(-10),
          Units.Degrees.of(180)
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

    public static final int SPINDEXER_MOTOR = 23;

    public static final int INTAKE_MOTOR = 24;
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

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true)
      .smartCurrentLimit(30, 30)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);
  }

  public static final class INTAKE_PIVOT {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(75);

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(30, 30)
      .openLoopRampRate(0.25)
      .voltageCompensation(12);
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

  public static final class FEEDER {

    public static final Dimensionless AXIS_MAX_SPEED = Units.Percent.of(100);

    public static final SparkBaseConfig FEEDER_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(20, 20)
      .openLoopRampRate(0.5)
      .voltageCompensation(12);
  }

  public static final class SHOOTER {

    public static final MechanismGearing GEARING = new MechanismGearing(
      GearBox.fromStages("1:1")
    );

    // Diameter of the flywheel.
    public static final Distance DIAMETER = Inches.of(4);
    // Mass of the flywheel.
    public static final Mass MASS = Pounds.of(2.717);
    // Maximum speed of the shooter.
    public static final AngularVelocity MAX_VELOCITY = RPM.of(5676);
    // Telemetry name and verbosity for the arm.
    public static final String NETWORK_KEY = "ShooterMech";

    public static final Current STALL_LIMIT = Amps.of(40);

    public static final SparkBaseConfig SHOOTER_BASE_CONFIG =
      new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) STALL_LIMIT.in(Amps), 40)
        .voltageCompensation(12);

    // TODO: PID, Feedforward, max angular acceleration still need tuned for mechanism
    public static final double P = 0.1;
    public static final double I = 0;
    public static final double D = 0;
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
      RotationsPerSecondPerSecond.of(50);

    public static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(0, 0.01, 0.01);

    public static final Dimensionless AXIS_MAX_SPEED = Percent.of(100);

    public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(
      Units.Inches.of(-1.566),
      Units.Inches.of(-9.199),
      new Rotation2d(Units.Degrees.of(-50))
    );
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

  public class FieldConstants {

    public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // AprilTag related constants
    public static final int aprilTagCount = FIELD_LAYOUT.getTags().size();
    public static final double aprilTagWidth =
      edu.wpi.first.math.util.Units.inchesToMeters(6.5);

    // Field dimensions
    public static final double fieldLength = FIELD_LAYOUT.getFieldLength();
    public static final double fieldWidth = FIELD_LAYOUT.getFieldWidth();

    /** Hub related constants */
    public static class Hub {

      // Dimensions
      public static final double width =
        edu.wpi.first.math.util.Units.inchesToMeters(47.0);
      public static final double height =
        edu.wpi.first.math.util.Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth =
        edu.wpi.first.math.util.Units.inchesToMeters(41.7);
      public static final double innerHeight =
        edu.wpi.first.math.util.Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation3d topCenterPoint = new Translation3d(
        FIELD_LAYOUT.getTagPose(26).get().getX() + width / 2.0,
        fieldWidth / 2.0,
        height
      );
      public static final Translation3d innerCenterPoint = new Translation3d(
        FIELD_LAYOUT.getTagPose(26).get().getX() + width / 2.0,
        fieldWidth / 2.0,
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
  }
}
