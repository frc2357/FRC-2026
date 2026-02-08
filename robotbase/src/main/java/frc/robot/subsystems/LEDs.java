package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDs extends SubsystemBase {

  private AddressableLED m_LEDs;
  private AddressableLEDBuffer m_LEDBuffer;
  LEDPattern m_rainbow = LEDPattern.rainbow(255, 50);
  LEDPattern m_scrollingRainbow;
  private Color m_lastColor = null;

  public static final Color MELTDOWN_ORANGE = new Color(50, 255, 0);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color BLUE = new Color(0, 0, 225);
  public static final Color RED = new Color(255, 0, 0);
  public static final Color PURPLE = new Color(255, 0, 255);

  // MELTDOWN_ORANGE color was used from previous years but doesn't show up in charts

  public LEDs() {
    m_LEDs = new AddressableLED(LED.kPort);
    m_LEDBuffer = new AddressableLEDBuffer(LED.kLength);

    m_LEDs.setData(m_LEDBuffer);
    m_LEDs.start();
  }

  private void setColor(Color color) {
    if (color == m_lastColor) {
      return;
    }

    for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, color);
    }

    m_LEDs.setData(m_LEDBuffer);
    m_lastColor = color;
  }

  public void setIntaking() {
    setColor(Color.kRed);
  }

  public void setShooting() {
    setColor(Color.kBlue);
  }

  public void setOuttaking() {
    setColor(Color.kGreen);
  }
}
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
