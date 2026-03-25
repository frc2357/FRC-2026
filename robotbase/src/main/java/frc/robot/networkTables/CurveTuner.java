package frc.robot.networkTables;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.InterpolatingTreeMap;
import java.util.Comparator;
import java.util.Map;

public class CurveTuner<
  K extends Measure<DistanceUnit>,
  V extends Measure<? extends Unit>
> extends InterpolatingTreeMap<K, V> {

  private String m_name;
  private int m_size = 0;

  public CurveTuner(
    String name,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator
  ) {
    super(inverseInterpolator, interpolator);
    m_name = name;
  }

  public CurveTuner(
    String name,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator,
    Comparator<K> comparator
  ) {
    super(inverseInterpolator, interpolator, comparator);
    m_name = name;
  }

  public void put(K key, V value) {
    super.put(key, value);
    m_size++;

    Preferences.initDouble(
      String.format(
        "%s/Setpoint %d: %.2f %s",
        m_name,
        m_size,
        key.magnitude(),
        key.unit().symbol()
      ),
      value.magnitude()
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

  public String getName() {
    return m_name;
  }
}
