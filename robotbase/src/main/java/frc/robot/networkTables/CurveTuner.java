package frc.robot.networkTables;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.InterpolatingTreeMap;
import java.util.Comparator;
import java.util.Map;

public class CurveTuner<
  K extends Measure<DistanceUnit>,
  V extends Measure<? extends Unit>
> extends InterpolatingTreeMap<K, V> {

  private String m_name;

  private SendableChooser<K> m_rowChooser = new SendableChooser<>();

  public CurveTuner(
    String name,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator
  ) {
    super(inverseInterpolator, interpolator);
    init(name);
  }

  public CurveTuner(
    String name,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator,
    Comparator<K> comparator
  ) {
    super(inverseInterpolator, interpolator, comparator);
    init(name);
  }

  public void init(String name) {
    m_name = name;

    m_rowChooser.onChange(val -> updateSelectedCurveIndex(val));
    SmartDashboard.putData(m_name, m_rowChooser);
  }

  private void updateSelectedCurveIndex(K key) {
    V value = get(key);
    SmartDashboard.putNumber(
      String.format("%s Value (%s)", m_name, value.unit().name()),
      value.magnitude()
    );
  }

  @SuppressWarnings("unchecked")
  public void updateCurveValues() {
    K key = m_rowChooser.getSelected();
    if (key == null) return;
    V previousValue = get(key);

    String smartDashboardKey = String.format(
      "%s Value (%s)",
      m_name,
      previousValue.unit().name()
    );
    double currentValueMagnitude = SmartDashboard.getNumber(
      smartDashboardKey,
      previousValue.magnitude()
    );
    put(key, (V) previousValue.unit().of(currentValueMagnitude));
  }

  public void put(K key, V value) {
    super.put(key, value);

    m_rowChooser.addOption(
      String.format("Distance: %.2f %s", key.magnitude(), key.unit().symbol()),
      key
    );
  }

  public void logCurrentValues() {
    System.out.println(m_name + " - Current Values");
    for (Map.Entry<K, V> entry : m_map.entrySet()) {
      System.out.println(
        String.format(
          "\tKey: %f %s - Value: %f %s",
          entry.getKey().magnitude(),
          entry.getKey().unit().name(),
          entry.getValue().magnitude(),
          entry.getValue().unit().name()
        )
      );
    }
    System.out.println();
  }
}
