package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Acceleration;
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
}
