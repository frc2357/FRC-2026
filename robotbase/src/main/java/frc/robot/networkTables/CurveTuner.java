package frc.robot.networkTables;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.InterpolatingTreeMap;
import java.util.Comparator;

public class CurveTuner<
  KU extends Unit,
  K extends Measure<KU>,
  VU extends Unit,
  V extends Measure<VU>
> extends InterpolatingTreeMap<K, V> {

  private String m_name;
  private KU m_keyConversionUnit;
  private VU m_valueConversionUnit;

  public CurveTuner(
    String name,
    KU keyConversionUnit,
    VU valueConversionUnit,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator
  ) {
    super(inverseInterpolator, interpolator);
    m_name = name;
    m_keyConversionUnit = keyConversionUnit;
    m_valueConversionUnit = valueConversionUnit;
  }

  public CurveTuner(
    String name,
    KU keyConversionUnit,
    VU valueConversionUnit,
    InverseInterpolator<K> inverseInterpolator,
    Interpolator<V> interpolator,
    Comparator<K> comparator
  ) {
    super(inverseInterpolator, interpolator, comparator);
    m_name = name;
    m_keyConversionUnit = keyConversionUnit;
    m_valueConversionUnit = valueConversionUnit;
  }

  public void put(K key, V value) {
    super.put(key, value);

    Preferences.initDouble(
      String.format(
        "%s/Setpoint: %.2f %s",
        m_name,
        key.in(m_keyConversionUnit),
        m_keyConversionUnit.symbol()
      ),
      value.in(m_valueConversionUnit)
    );
  }

  @SuppressWarnings("unchecked")
  public void updateCurveValue(K key, double newMagnitude) {
    super.put(key, (V) m_valueConversionUnit.of(newMagnitude));
  }

  @SuppressWarnings("unchecked")
  public void updateCurveValue(K key) {
    V previousValue = get(key);

    String prefKey = String.format(
      "%s/Setpoint: %.2f %s",
      m_name,
      key.in(m_keyConversionUnit),
      m_keyConversionUnit.symbol()
    );
    double preferencesMagnitude = Preferences.getDouble(
      prefKey,
      previousValue.in(m_valueConversionUnit)
    );

    super.put(key, (V) m_valueConversionUnit.of(preferencesMagnitude));
  }

  public void updateCurveValues() {
    for (K key : m_map.keySet()) {
      updateCurveValue(key);
    }
  }

  public String getName() {
    return m_name;
  }
}
