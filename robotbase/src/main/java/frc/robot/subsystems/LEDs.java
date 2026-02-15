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

    m_LEDBuffer = new AddressableLEDBuffer(Constants.LED.kLength);
    m_LEDs.setLength(m_LEDBuffer.getLength());
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  // Robot.led.setLEDPattern(pattern)
  public void setLEDPattern(LEDPattern newPattern) {
    newPattern.applyTo(m_LEDBuffer);
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  private void ledPeriodic() {
    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  // private void setColor(Color color) {
  // if (color == m_lastColor) {
  // return;
  // }

  // for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
  // m_LEDBuffer.setLED(i, color);
  //}

  //m_LEDs.setData(m_LEDBuffer);
  //m_lastColor = color;
  //}
}
// new SetLEDCommand(Color.kRed)
//chart for different colors
//Color 	Red (0-255)	Green (0-255)	Blue (0-255)	Hex Code
//Red	255	0	0	#FF0000
//Green	0	255	0	#00FF00
//Blue	0	0	255	#0000FF
//White	255	255	255	#FFFFFF
//Yellow	255	255	0	#FFFF00
//Cyan	0	255	255	#00FFFF
//Magenta	255	0	255	#FF00FF
//Orange	255	165	0	#FFA500
//Purple	128	0	128	#800080
//Pink	255	192	203	#FFC0CB
//Teal	0	128	128	#008080
//Amber	255	191	0	#FFBF00
