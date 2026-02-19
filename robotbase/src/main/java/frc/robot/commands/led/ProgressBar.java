package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ProgressBar extends Command {

  private double m_progressSpeed;
  private double cullLength;

  public ProgressBar(Time loopTime) {
    addRequirements(Robot.led);
    m_progressSpeed = (86 / loopTime.in(Seconds)) / 50;
    cullLength = 0;
  }

  public void execute() {
    cullLength += m_progressSpeed;

    if (cullLength > LED.kLength) {
      cullLength = 0;
    }

    for (int i = 0; i < LED.kLength; i++) {
      if (i < cullLength) {
        new Color(255, 0, 0);
      } else {
        new Color(0, 0, 0);
        //set black
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  //send buffer to led

  @Override
  public void end(boolean interrupted) {}
}
