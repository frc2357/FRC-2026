package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
}
