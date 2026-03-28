package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.networkTables.CurveTuner;
import frc.robot.util.InterpolationUtil;

public class ShooterCurveManager {

  private final String shooterOffsetKey = "shooter offset rps";
  private final String hoodOffsetKey = "hood offset rps";

  private final CurveTuner<
    DistanceUnit,
    Distance,
    AngularVelocityUnit,
    AngularVelocity
  > passingShooterCurve = new CurveTuner<>(
    "Passing Shooter Curve",
    Inches,
    RotationsPerSecond,
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<
    DistanceUnit,
    Distance,
    AngleUnit,
    Angle
  > passingHoodCurve = new CurveTuner<>(
    "Passing Hood Curve",
    Inches,
    Degrees,
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<
    DistanceUnit,
    Distance,
    AngularVelocityUnit,
    AngularVelocity
  > scoringShooterCurve = new CurveTuner<>(
    "Shooter Curve",
    Inches,
    RotationsPerSecond,
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<
    DistanceUnit,
    Distance,
    AngleUnit,
    Angle
  > scoringHoodCurve = new CurveTuner<>(
    "Hood Curve",
    Inches,
    Degrees,
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<
    DistanceUnit,
    Distance,
    TimeUnit,
    Time
  > timeOfFlightCurve = new CurveTuner<>(
    "ToF Curve",
    Inches,
    Seconds,
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  public static final class SHOT_POINTS {

    public static final Distance HUB = Inches.of(43);
    public static final Distance POINT_2 = Inches.of(75);
    public static final Distance POINT_3 = Inches.of(100);
    public static final Distance TRENCH = Inches.of(127);
    public static final Distance POINT_5 = Inches.of(150);
    public static final Distance POINT_6 = Inches.of(175);
    public static final Distance OUTPOST_CORNER = Inches.of(205);
  }

  public static final Distance[] SHOT_POINT_ARRAY = {
    SHOT_POINTS.HUB,
    SHOT_POINTS.POINT_2,
    SHOT_POINTS.POINT_3,
    SHOT_POINTS.TRENCH,
    SHOT_POINTS.POINT_5,
    SHOT_POINTS.POINT_6,
    SHOT_POINTS.OUTPOST_CORNER,
  };

  public static final class PASS_POINTS {

    public static final Distance CLOSEST = Inches.of(200);
    public static final Distance CEILING = Inches.of(400);
    public static final Distance FURTHERST = Inches.of(
      Constants.FieldConstants.fieldLength
    );
  }

  public ShooterCurveManager() {
    initializeCurves();

    SmartDashboard.putNumber(shooterOffsetKey, 0);
    SmartDashboard.putNumber(hoodOffsetKey, 0);
  }

  private void initializeCurves() {
    // Passing shooter
    passingShooterCurve.put(PASS_POINTS.CLOSEST, RotationsPerSecond.of(50));
    // passingShooterCurve.put(PASS_POINTS.CEILING, RotationsPerSecond.of(95));
    passingShooterCurve.put(PASS_POINTS.FURTHERST, RotationsPerSecond.of(95));

    // Passing hood
    passingHoodCurve.put(PASS_POINTS.CLOSEST, Degrees.of(18));
    passingHoodCurve.put(PASS_POINTS.CEILING, Degrees.of(34));
    passingHoodCurve.put(PASS_POINTS.FURTHERST, Degrees.of(34));

    scoringShooterCurve.put(SHOT_POINTS.HUB, RotationsPerSecond.of(43));
    scoringShooterCurve.put(SHOT_POINTS.POINT_2, RotationsPerSecond.of(46));
    scoringShooterCurve.put(SHOT_POINTS.POINT_3, RotationsPerSecond.of(47));
    scoringShooterCurve.put(SHOT_POINTS.TRENCH, RotationsPerSecond.of(49));
    scoringShooterCurve.put(SHOT_POINTS.POINT_5, RotationsPerSecond.of(52));
    scoringShooterCurve.put(SHOT_POINTS.POINT_6, RotationsPerSecond.of(54));
    scoringShooterCurve.put(
      SHOT_POINTS.OUTPOST_CORNER,
      RotationsPerSecond.of(58)
    );

    // Scoring hood
    scoringHoodCurve.put(SHOT_POINTS.HUB, Degrees.of(1));
    scoringHoodCurve.put(SHOT_POINTS.POINT_2, Degrees.of(4));
    scoringHoodCurve.put(SHOT_POINTS.POINT_3, Degrees.of(6));
    scoringHoodCurve.put(SHOT_POINTS.TRENCH, Degrees.of(10.5));
    scoringHoodCurve.put(SHOT_POINTS.POINT_5, Degrees.of(13));
    scoringHoodCurve.put(SHOT_POINTS.POINT_6, Degrees.of(17));
    scoringHoodCurve.put(SHOT_POINTS.OUTPOST_CORNER, Degrees.of(18));

    // Time of flight
    timeOfFlightCurve.put(SHOT_POINTS.HUB, Seconds.of(1.005));
    timeOfFlightCurve.put(SHOT_POINTS.POINT_2, Seconds.of(1.068));
    timeOfFlightCurve.put(SHOT_POINTS.POINT_3, Seconds.of(1.168));
    timeOfFlightCurve.put(SHOT_POINTS.TRENCH, Seconds.of(1.132));
    timeOfFlightCurve.put(SHOT_POINTS.POINT_5, Seconds.of(1.032));
    timeOfFlightCurve.put(SHOT_POINTS.POINT_6, Seconds.of(1.138));
    timeOfFlightCurve.put(SHOT_POINTS.OUTPOST_CORNER, Seconds.of(1.18));

    updateCurveValues();
  }

  public AngularVelocity getScoringShooterVelocity(Distance distance) {
    return scoringShooterCurve
      .get(distance)
      .plus(
        RotationsPerSecond.of(SmartDashboard.getNumber(shooterOffsetKey, 0))
      );
  }

  public Angle getScoringHoodAngle(Distance distance) {
    return scoringHoodCurve
      .get(distance)
      .plus(Degrees.of(SmartDashboard.getNumber(hoodOffsetKey, 0)));
  }

  public AngularVelocity getPassingShooterVelocity(Distance distance) {
    return passingShooterCurve.get(distance);
  }

  public Angle getPassingHoodAngle(Distance distance) {
    return passingHoodCurve.get(distance);
  }

  public Time getTimeOfFlight(Distance distance) {
    return timeOfFlightCurve.get(distance);
  }

  public void updateCurveValues() {
    scoringShooterCurve.updateCurveValues();
    scoringHoodCurve.updateCurveValues();
    passingShooterCurve.updateCurveValues();
    passingHoodCurve.updateCurveValues();
    timeOfFlightCurve.updateCurveValues();
  }

  public CurveTuner<
    DistanceUnit,
    Distance,
    AngularVelocityUnit,
    AngularVelocity
  > getScoringShooterCurve() {
    return scoringShooterCurve;
  }

  public CurveTuner<
    DistanceUnit,
    Distance,
    AngleUnit,
    Angle
  > getScoringHoodCurve() {
    return scoringHoodCurve;
  }
}
