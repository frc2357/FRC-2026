package frc.robot.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ShiftTimer.Shift;
import frc.robot.ShiftTimer.ShiftInfo;

public class ShiftWarning {

  private boolean m_hasWarned = false;
  private Shift m_currentShift = Shift.DISABLED;

  private boolean shouldWarn() {
    ShiftInfo shiftInfo = Robot.shiftTimer.getShiftInfo();

    if (m_currentShift != shiftInfo.shift()) {
      m_hasWarned = false;
      m_currentShift = shiftInfo.shift();
    }

    boolean shouldWarn = isHubAboutToActivate(shiftInfo) && !m_hasWarned;
    if (shouldWarn) {
      m_hasWarned = true;
      return true;
    }
    return false;
  }

  private boolean isHubAboutToActivate(ShiftInfo shiftInfo) {
    return (
      !shiftInfo.isHubActive() &&
      shiftInfo
        .timeRemaining()
        .lt(Constants.SCORING.TIME_TO_WARN_FOR_ACTIVE_HUB)
    );
  }

  public Trigger warn() {
    return new Trigger(this::shouldWarn);
  }
}
