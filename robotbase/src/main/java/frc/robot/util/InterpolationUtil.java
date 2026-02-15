package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class InterpolationUtil {

  public static double InverseInterpolate(
    Distance startValue,
    Distance endValue,
    Distance q
  ) {
    return InverseInterpolator.forDouble().inverseInterpolate(
      startValue.in(Inches),
      endValue.in(Inches),
      q.in(Inches)
    );
  }

  public static Angle Interpolate(Angle startValue, Angle endValue, double q) {
    return Degrees.of(
      Interpolator.forDouble().interpolate(
        startValue.in(Degrees),
        endValue.in(Degrees),
        q
      )
    );
  }

  public static AngularVelocity Interpolate(
    AngularVelocity startValue,
    AngularVelocity endValue,
    double q
  ) {
    return RPM.of(
      Interpolator.forDouble().interpolate(
        startValue.in(RPM),
        endValue.in(RPM),
        q
      )
    );
  }
}
