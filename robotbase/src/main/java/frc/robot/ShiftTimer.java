package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShiftTimer {

  public record ShiftInfo(
    Shift shift,
    boolean isHubActive,
    Time timeRemaining
  ) {}

  public enum Shift {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED,
  }

  private static Timer shiftTimer = new Timer();

  // Times relative to teleop (start of teleop = 0)
  private static final Time[] teleopShiftStartTimes = {
    Seconds.of(0),
    Seconds.of(10.0),
    Seconds.of(35.0),
    Seconds.of(60.0),
    Seconds.of(85.0),
    Seconds.of(110.0),
  };
  private static final Time[] teleopShiftEndTimes = {
    Seconds.of(10.0),
    Seconds.of(35.0),
    Seconds.of(60.0),
    Seconds.of(85.0),
    Seconds.of(110.0),
    Seconds.of(140.0),
  };

  private static final Shift[] shifts = Shift.values();
  private static final boolean[] autoLoseSchedule = {
    true,
    true,
    false,
    true,
    false,
    true,
  };
  private static final boolean[] autoWinSchedule = {
    true,
    false,
    true,
    false,
    true,
    true,
  };

  public ShiftTimer() {
    // Reset on auto start so we calculate remaining shift time for auto
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::startTimer));
    // Reset on teleop start so we calculate remaining shift time from teleop start
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(this::startTimer));

    // Reset every 140 seconds
    new Trigger(DriverStation::isTeleop)
      .and(() -> shiftTimer.hasElapsed(Constants.SCORING.TELEOP_LENGTH))
      .onTrue(Commands.runOnce(this::startTimer));
  }

  public void startTimer() {
    shiftTimer.restart();
  }

  public Alliance getWinningAlliance() {
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Red;
      } else if (character == 'B') {
        return Alliance.Blue;
      }
    }
    // Default to blue
    return Alliance.Blue;
  }

  public boolean[] getSchedule() {
    Alliance alliance = getWinningAlliance();

    if (alliance == Robot.alliance) {
      return autoWinSchedule;
    } else {
      return autoLoseSchedule;
    }
  }

  public ShiftInfo getShiftInfo() {
    Shift shift = Shift.DISABLED;
    boolean isHubActive = false;
    Time timeRemaining = Seconds.of(0);
    Time currentTime = Seconds.of(shiftTimer.get());

    if (DriverStation.isAutonomous()) {
      shift = Shift.AUTO;
      isHubActive = true;
      timeRemaining = Constants.SCORING.AUTO_LENGTH.minus(currentTime);
    }

    if (DriverStation.isTeleop()) {
      boolean[] schedule = getSchedule();

      for (int i = 0; i < teleopShiftStartTimes.length; i++) {
        if (
          currentTime.gte(teleopShiftStartTimes[i]) &&
          currentTime.lt(teleopShiftEndTimes[i])
        ) {
          shift = shifts[i];
          isHubActive = schedule[i];
          timeRemaining = teleopShiftEndTimes[i].minus(currentTime);
        }
      }
    }

    return new ShiftInfo(shift, isHubActive, timeRemaining);
  }

  public boolean isHubAboutToActivate() {
    ShiftInfo shiftInfo = getShiftInfo();

    return (
      !shiftInfo.isHubActive &&
      shiftInfo.timeRemaining.lt(Constants.SCORING.TIME_TO_WARN_FOR_ACTIVE_HUB)
    );
  }
}
