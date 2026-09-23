package subsystems;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HOOD;
import frc.robot.subsystems.hood.Hood;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;


@DisplayName("Hood Subsystem Tests")
public class HoodTest {

  // ============ TEST CONSTANTS ============
  private static final Angle SMALL_ANGLE = Degrees.of(5);
  private static final Angle MID_ANGLE = Degrees.of(15);
  private static final Angle LARGE_ANGLE = Degrees.of(30);
  
  private static final int CONVERGENCE_WAIT_CYCLES = 50;
  private static final int NORMAL_WAIT_CYCLES = 10;
  
  // TOLERANCE: Allow 1.5 degrees. This accounts for encoder resolution and PID tuning.
  private static final double ANGLE_TOLERANCE_DEGREES = 1.5;
  
  // ============ CLASS VARIABLES ============
  private static Hood hood;
  private static CommandScheduler scheduler;

  @BeforeAll
  public static void setUpClass() {
    // Initialize HAL once for the entire test class
    HAL.initialize(500, 0);
    
    // Get the CommandScheduler singleton instance
    scheduler = CommandScheduler.getInstance();
  
    // Create Hood once - it will be reused for all tests
    hood = new Hood();
  }

  @BeforeEach
  public void setUp() {
    // Clear the scheduler and reset the hood to a known state to keep tests isolated.
    scheduler.cancelAll();
    hood.stop();
    hood.goHome();
    runSchedulerCycles(5);
  }

  @AfterAll
  public static void tearDownClass() {
    // Clean up after all tests
    if (hood != null) {
      hood.stop();
    }
    scheduler.close();
  }

  // ============ INITIALIZATION & CONFIGURATION TESTS ============

  @Test
  @DisplayName("Hood subsystem initializes without errors")
  public void testHoodInitialization() {
    assertNotNull(hood, "Hood should not be null");
    assertNotNull(hood.getHood(), "Hood mechanism should not be null");
  }

  @Test
  @DisplayName("Initial angle can be retrieved")
  public void testGetAngle() {
    Angle currentAngle = hood.getAngle();
    assertNotNull(currentAngle, "Current angle should not be null");
    
    double angleDegrees = currentAngle.in(Degrees);
    double lowerLimit = HOOD.LOWER_ANGLE_LIMIT.in(Degrees);
    double upperLimit = HOOD.UPPER_ANGLE_LIMIT.in(Degrees);
    
    System.out.println("Initial angle: " + angleDegrees + " degrees");
    System.out.println("Soft limits: [" + lowerLimit + ", " + upperLimit + "]");
    
    assertTrue(
      angleDegrees >= lowerLimit - 1.5 && angleDegrees <= upperLimit + 1.5,
      "Initial angle should be within soft limits (with 1.5° tolerance). " +
      "Got: " + angleDegrees + "°, Limits: [" + lowerLimit + "°, " + upperLimit + "°]. " +
      "This may indicate an encoder offset issue."
    );
  }

  // ============ COMMAND CREATION TESTS ============

  @Test
  @DisplayName("setAngle(Supplier<Angle>) runs without error")
  public void testSetAngleWithSupplier() {
    Angle targetAngle = Rotations.of(0.5);

    assertDoesNotThrow(() -> hood.setAngle(targetAngle), "setAngle with Supplier should not throw");
  }

  @Test
  @DisplayName("goHome() runs without error")
  public void testGoHome() {
   assertDoesNotThrow(() -> hood.goHome(), "goHome should not throw");
  }

  @Test
  @DisplayName("set() runs without error")
  public void testSetSpeed() {
    Dimensionless speed = Percent.of(50);
    assertDoesNotThrow(() -> hood.set(speed), "set should not throw");
  }

  @Test
  @DisplayName("axisSpeed() runs without error")
  public void testAxisSpeed() {
    Dimensionless axisValue = Percent.of(75);
   assertDoesNotThrow(() -> hood.axisSpeed(axisValue), "axisSpeed should not throw");
  }

  @Test
  @DisplayName("axisSpeed() with zero value runs without error")
  public void testAxisSpeedZero() {
    Dimensionless zeroAxis = Percent.of(0);
    assertDoesNotThrow(() -> hood.axisSpeed(zeroAxis), "axisSpeed with zero should not throw");
  }

  @Test
  @DisplayName("axisSpeed() accepts reverse motion")
  public void testAxisSpeedReverse() {
    Dimensionless reverseAxis = Percent.of(-25);
    assertDoesNotThrow(() -> hood.axisSpeed(reverseAxis), "axisSpeed with a negative value should not throw");
  }

  @Test
  @DisplayName("set(double) accepts negative manual duty cycle")
  public void testSetNegativeDutyCycle() {
    assertDoesNotThrow(() -> hood.set(-0.25), "set should accept negative duty cycle values");
  }

  @Test
  @DisplayName("stop() runs without error")
  public void testStopCommand() {
    assertDoesNotThrow(() -> hood.stop(), "stopCommand should not throw");
  }

  // ============ BOUNDARY CONDITION TESTS ============

  @Test
  @DisplayName("Hood respects lower soft limit")
  public void testLowerSoftLimit() {
    Angle lowerLimit = HOOD.LOWER_ANGLE_LIMIT;
    hood.setAngle(lowerLimit);
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    System.out.println("Lower limit test - Target: " + lowerLimit.in(Degrees) + 
                       ", Actual: " + currentAngle.in(Degrees));
    
    assertTrue(
      currentAngle.in(Degrees) >= lowerLimit.in(Degrees) - ANGLE_TOLERANCE_DEGREES,
      "Hood should not go below lower limit"
    );
  }

  @Test
  @DisplayName("Hood respects upper soft limit")
  public void testUpperSoftLimit() {
    Angle upperLimit = HOOD.UPPER_ANGLE_LIMIT;
    hood.setAngle(upperLimit);
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    System.out.println("Upper limit test - Target: " + upperLimit.in(Degrees) + 
                       ", Actual: " + currentAngle.in(Degrees));
    
    assertTrue(
      currentAngle.in(Degrees) <= upperLimit.in(Degrees) + ANGLE_TOLERANCE_DEGREES,
      "Hood should not exceed upper limit"
    );
  }

  @Test
  @DisplayName("Hood clamps setpoints beyond limits")
  public void testSetpointClamping() {
    // Attempt to set angle beyond upper limit
    Angle beyondLimit = HOOD.UPPER_ANGLE_LIMIT.plus(Degrees.of(10));
    hood.setAngle(beyondLimit);
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    System.out.println("Clamping test - Beyond limit: " + beyondLimit.in(Degrees) + 
                       ", Actual: " + currentAngle.in(Degrees));
    
    assertTrue(
      currentAngle.in(Degrees) <= HOOD.UPPER_ANGLE_LIMIT.in(Degrees) + ANGLE_TOLERANCE_DEGREES,
      "Hood should clamp angles beyond upper limit"
    );
  }

  // ============ CLOSED-LOOP CONTROL & CONVERGENCE TESTS ============

  @Disabled
  @Test
  @DisplayName("Hood converges to small angle setpoint")
  public void testConvergenceSmallAngle() {
    Angle targetAngle = SMALL_ANGLE;
    hood.setAngle(targetAngle);
    
    runSchedulerCycles(1000);
    
    Angle currentAngle = hood.getAngle();
    double angleDifference = Math.abs(
      currentAngle.in(Degrees) - targetAngle.in(Degrees)
    );
    
    System.out.println("Small angle convergence - Target: " + targetAngle.in(Degrees) + 
                       "°, Actual: " + currentAngle.in(Degrees) + 
                       "°, Difference: " + angleDifference + "°");
    
    assertTrue(
      angleDifference <= ANGLE_TOLERANCE_DEGREES,
      "Hood should converge to small angle within tolerance. Difference: " + 
      angleDifference + "° (tolerance: " + ANGLE_TOLERANCE_DEGREES + "°), " +
      "Target: " + targetAngle.in(Degrees) + "°, Actual: " + currentAngle.in(Degrees) + "°"
    );
  }

  @Disabled
  @Test
  @DisplayName("Hood converges to mid angle setpoint")
  public void testConvergenceMidAngle() {
    Angle targetAngle = MID_ANGLE;
    hood.setAngle(targetAngle);
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    double angleDifference = Math.abs(
      currentAngle.in(Degrees) - targetAngle.in(Degrees)
    );
    
    System.out.println("Mid angle convergence - Target: " + targetAngle.in(Degrees) + 
                       "°, Actual: " + currentAngle.in(Degrees) + 
                       "°, Difference: " + angleDifference + "°");
    
    assertTrue(
      angleDifference <= ANGLE_TOLERANCE_DEGREES,
      "Hood should converge to mid angle within tolerance. Difference: " + 
      angleDifference + "° (tolerance: " + ANGLE_TOLERANCE_DEGREES + "°), " +
      "Target: " + targetAngle.in(Degrees) + "°, Actual: " + currentAngle.in(Degrees) + "°"
    );
  }

  @Disabled
  @Test
  @DisplayName("Hood converges to large angle setpoint")
  public void testConvergenceLargeAngle() {
    Angle targetAngle = LARGE_ANGLE;
    hood.setAngle(targetAngle);
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    double angleDifference = Math.abs(
      currentAngle.in(Degrees) - targetAngle.in(Degrees)
    );
    
    System.out.println("Large angle convergence - Target: " + targetAngle.in(Degrees) + 
                       "°, Actual: " + currentAngle.in(Degrees) + 
                       "°, Difference: " + angleDifference + "°");
    
    assertTrue(
      angleDifference <= ANGLE_TOLERANCE_DEGREES,
      "Hood should converge to large angle within tolerance. Difference: " + 
      angleDifference + "° (tolerance: " + ANGLE_TOLERANCE_DEGREES + "°), " +
      "Target: " + targetAngle.in(Degrees) + "°, Actual: " + currentAngle.in(Degrees) + "°"
    );
  }

  @Test
  @DisplayName("No overshoot when angle already at target")
  public void testNoOscillationAtSetpoint() {
    Angle targetAngle = MID_ANGLE;
    hood.setAngle(targetAngle);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES); // Converge first
    
    // Verify it stays at setpoint without oscillating
    double[] angles = new double[NORMAL_WAIT_CYCLES];
    for (int i = 0; i < NORMAL_WAIT_CYCLES; i++) {
      angles[i] = hood.getAngle().in(Degrees);
      runSchedulerCycles(1);
    }
    
    // Check standard deviation is low (no oscillation)
    double mean = 0;
    for (double angle : angles) mean += angle;
    mean /= angles.length;
    
    double variance = 0;
    for (double angle : angles) {
      variance += Math.pow(angle - mean, 2);
    }
    double stdDev = Math.sqrt(variance / angles.length);
    
    System.out.println("Oscillation test - Mean: " + mean + "°, StdDev: " + stdDev + "°");
    
    assertTrue(
      stdDev < 1.5,
      "Hood should not oscillate at setpoint. StdDev: " + stdDev + "°"
    );
  }

  // ============ COMMAND INTERRUPTION & CLEANUP TESTS ============

  @Test
  @DisplayName("stopMotor() stops motion immediately")
  public void testStopMotorStopsMotion() {
    // Start moving
    hood.setAngle(LARGE_ANGLE);
    runSchedulerCycles(5);
    
    // Stop
    hood.stop();
    
    // Verify motion stops
    Angle angleAfterStop1 = hood.getAngle();
    runSchedulerCycles(5);
    Angle angleAfterStop2 = hood.getAngle();
    
    double angleDifference = Math.abs(
      angleAfterStop2.in(Degrees) - angleAfterStop1.in(Degrees)
    );
    
    System.out.println("Stop test - Angle before: " + angleAfterStop1.in(Degrees) + 
                       "°, After: " + angleAfterStop2.in(Degrees) + 
                       "°, Difference: " + angleDifference + "°");
    
    assertTrue(
      angleDifference < 1.0,
      "Hood should stop moving. Difference: " + angleDifference + "°"
    );
  }

  // ============ ENCODER FEEDBACK & TELEMETRY TESTS ============

  @Test
  @DisplayName("Encoder feedback is consistent after movement")
  public void testEncoderConsistency() {
    hood.setAngle(MID_ANGLE);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle angle1 = hood.getAngle();
    runSchedulerCycles(5); // Idle
    Angle angle2 = hood.getAngle();
    
    double difference = Math.abs(angle1.in(Degrees) - angle2.in(Degrees));
    System.out.println("Encoder consistency - Angle 1: " + angle1.in(Degrees) + 
                       "°, Angle 2: " + angle2.in(Degrees) + 
                       "°, Difference: " + difference + "°");
    
    assertTrue(
      difference < 0.5,
      "Encoder feedback should be consistent. Difference: " + difference + "°"
    );
  }

  @Test
  @DisplayName("Telemetry updates without errors in cycles")
  public void testTelemetryUpdates() {
    assertDoesNotThrow(() -> {
      for (int i = 0; i < 10; i++) {
        hood.periodic();
        hood.simulationPeriodic();
      }
    }, "Telemetry should update without errors");
  }

  // ============ MOTION PROFILE TESTS ============

  @Test
  @DisplayName("Motion is smooth (acceleration ramps)")
  public void testSmoothAcceleration() {
    hood.setAngle(LARGE_ANGLE);
    
    double[] angles = new double[CONVERGENCE_WAIT_CYCLES];
    for (int i = 0; i < CONVERGENCE_WAIT_CYCLES; i++) {
      angles[i] = hood.getAngle().in(Degrees);
      runSchedulerCycles(1);
    }
    
    // Check that angles are mostly monotonically increasing (smooth ramp)
    int monotonicCount = 0;
    for (int i = 1; i < angles.length; i++) {
      if (angles[i] >= angles[i-1]) {
        monotonicCount++;
      }
    }
    
    System.out.println("Smooth acceleration - Monotonic count: " + monotonicCount + 
                       " / " + (CONVERGENCE_WAIT_CYCLES - 1));
    
    // Allow some irregularities, but mostly should be smooth
    assertTrue(
      monotonicCount > (CONVERGENCE_WAIT_CYCLES * 0.75),
      "Motion should be smooth and mostly monotonic. Monotonic count: " + monotonicCount
    );
  }

  @Test
  @DisplayName("Rapid setpoint changes are handled gracefully")
  public void testRapidSetpointChanges() {
    Angle[] targets = {SMALL_ANGLE, LARGE_ANGLE, MID_ANGLE, SMALL_ANGLE};
    
    assertDoesNotThrow(() -> {
      for (Angle target : targets) {
        hood.setAngle(target);
        runSchedulerCycles(10);
      }
    }, "Rapid setpoint changes should not cause errors");
  }

  // ============ MULTI-COMMAND SEQUENCING TESTS ============

  @Test
  @DisplayName("Multiple angle setpoints can be set")
  public void testMultipleAngleSetpoints() {
    Angle angle1 = SMALL_ANGLE;
    Angle angle2 = MID_ANGLE;
    Angle angle3 = LARGE_ANGLE;
    
    assertDoesNotThrow(() -> {
      hood.setAngle(angle1);
      hood.setAngle(angle2);
      hood.setAngle(angle3);
    });
  }

  @Test
  @DisplayName("Hood can return to home after a manual override")
  public void testHomeRecoveryAfterManualOverride() {
    hood.setAngle(LARGE_ANGLE);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);

    hood.set(-0.25);
    runSchedulerCycles(5);
    hood.stop();
    hood.goHome();
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);

    Angle currentAngle = hood.getAngle();
    Angle homeAngle = HOOD.SETPOINTS.HOME;
    double difference = Math.abs(currentAngle.in(Degrees) - homeAngle.in(Degrees));

    assertTrue(
      difference <= ANGLE_TOLERANCE_DEGREES * 2,
      "Hood should recover to home after manual override. Difference: " +
      difference + "° (tolerance: " + ANGLE_TOLERANCE_DEGREES * 2 + "°)"
    );
  }

  @Disabled
  @Test
  @DisplayName("Command sequencing works correctly")
  public void testCommandSequencing() {
    Angle angle1 = SMALL_ANGLE;
    Angle angle2 = LARGE_ANGLE;

    hood.setAngle(angle1);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle afterCmd1 = hood.getAngle();
    hood.setAngle(angle2);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    double difference = Math.abs(afterCmd1.in(Degrees) - angle1.in(Degrees));
    
    System.out.println("Command sequencing - Target: " + angle1.in(Degrees) + 
                       "°, Actual: " + afterCmd1.in(Degrees) + 
                       "°, Angle2: " + angle2.in(Degrees) +
                       "°, Difference: " + difference + "°");
    
    assertTrue(
      difference <= ANGLE_TOLERANCE_DEGREES,
      "First command should bring hood to angle1. Difference: " + difference + 
      "° (tolerance: " + ANGLE_TOLERANCE_DEGREES + "°)"
    );
  }

  @Test
  @DisplayName("Home command brings hood to home position")
  public void testHomeCommandIntegration() {
    // Move away from home
    hood.setAngle(LARGE_ANGLE);
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    // Go home
    hood.goHome();
    
    runSchedulerCycles(CONVERGENCE_WAIT_CYCLES);
    
    Angle currentAngle = hood.getAngle();
    Angle homeAngle = HOOD.SETPOINTS.HOME;
    double difference = Math.abs(currentAngle.in(Degrees) - homeAngle.in(Degrees));
    
    System.out.println("Home command - Target: " + homeAngle.in(Degrees) + 
                       "°, Actual: " + currentAngle.in(Degrees) + 
                       "°, Difference: " + difference + "°");
    
    assertTrue(
      difference <= ANGLE_TOLERANCE_DEGREES,
      "Hood should return to home position. Difference: " + difference + 
      "° (tolerance: " + ANGLE_TOLERANCE_DEGREES + "°), " +
      "Target: " + homeAngle.in(Degrees) + "°, Actual: " + currentAngle.in(Degrees) + "°"
    );
  }

  // ============ HELPER METHODS ============

  /**
   * Runs the scheduler for multiple cycles and updates hood telemetry.
   * 
   * @param cycles Number of scheduler cycles to run
   */
  private static void runSchedulerCycles(int cycles) {
    for (int i = 0; i < cycles; i++) {
      //scheduler.run();
      hood.periodic();
      hood.simulationPeriodic();
    }
  }

}

