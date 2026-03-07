package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED;
import frc.robot.Robot;

public class LEDs extends SubsystemBase {

  public AddressableLED m_LEDs;
  public AddressableLEDBuffer m_LEDBuffer;

  public LEDs() {
    m_LEDs = new AddressableLED(LED.kPort);

    m_LEDBuffer = new AddressableLEDBuffer(LED.kLength);
    m_LEDs.setLength(m_LEDBuffer.getLength());
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  public void setLEDPattern(LEDPattern newPattern) {
    newPattern.applyTo(m_LEDBuffer);
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  public void ProgressBar(LEDPattern newPattern) {
    newPattern.applyTo(m_LEDBuffer);
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  private void ledPeriodic() {
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  // for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
  // m_LEDBuffer.setLED(i, color);

  //m_LEDs.setData(m_LEDBuffer);
  //m_lastColor = color;
}
