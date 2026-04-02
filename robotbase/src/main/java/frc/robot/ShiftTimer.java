package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SHIFT;

public class ShiftTimer implements Sendable {

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

  private static final Shift[] shifts = Shift.values();

  public ShiftTimer() {
    // Reset on auto start so we calculate remaining shift time for auto
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::startTimer));
    // Reset on teleop start so we calculate remaining shift time from teleop start
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(this::startTimer));
    // Reset every 140 seconds
    new Trigger(DriverStation::isTeleop)
      .and(() -> shiftTimer.hasElapsed(SHIFT.TELEOP_LENGTH))
      .onTrue(Commands.runOnce(this::startTimer));
  }

  private void startTimer() {
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
      return SHIFT.AUTO_WIN_SCHEDULE;
    } else {
      return SHIFT.AUTO_LOSE_SCHEDULE;
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
      timeRemaining = SHIFT.AUTO_LENGTH.minus(currentTime);
    }

    if (DriverStation.isTeleopEnabled()) {
      boolean[] schedule = getSchedule();

      for (int i = 0; i < SHIFT.TELEOP_SHIFT_START_TIMES.length; i++) {
        if (
          currentTime.gte(SHIFT.TELEOP_SHIFT_START_TIMES[i]) &&
          currentTime.lt(SHIFT.TELEOP_SHIFT_END_TIMES[i])
        ) {
          shift = shifts[i];
          isHubActive = schedule[i];
          timeRemaining = SHIFT.TELEOP_SHIFT_END_TIMES[i].minus(currentTime);
        }
      }
    }

    return new ShiftInfo(shift, isHubActive, timeRemaining);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ShiftTimer");

    // 2. Add the requested properties
    builder.addBooleanProperty(
      "Is Hub Active",
      () -> getShiftInfo().isHubActive(),
      null
    );
    builder.addStringProperty(
      "Current Shift",
      () -> getShiftInfo().shift().toString(),
      null
    );

    // Optional: Add time remaining since you already have the logic
    builder.addDoubleProperty(
      "Shift Time Remaining",
      () -> getShiftInfo().timeRemaining().in(Seconds),
      null
    );
  }
}
