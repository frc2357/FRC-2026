package frc.robot.commands.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs;

public class SetLEDPatternCommand extends Command {

  private LEDPattern m_LEDPattern;

  public SetLEDPatternCommand(LEDPattern ledpattern) {
    addRequirements(Robot.led);
    m_LEDPattern = ledpattern;
  }

  @Override
  public void execute() {
    Robot.led.setLEDPattern(m_LEDPattern);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
