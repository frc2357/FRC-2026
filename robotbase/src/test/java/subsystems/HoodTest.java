package subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;

public class HoodTest {

  // Static Hood instance - created once, reused for all tests
  private static Hood hood;

  @BeforeAll
  public static void setUpClass() {
    // Initialize HAL once for the entire test class
    HAL.initialize(500, 0);
    
    // Create Hood once - it will be reused for all tests
    hood = new Hood();
  }

  @AfterAll
  public static void tearDownClass() {
    // Clean up after all tests
    if (hood != null) {
      hood.stopMotor();
    }
  }

  // ============ INITIALIZATION TESTS ============

  @Test
  public void testHoodInitialization() {
    assertNotNull(hood);
    assertNotNull(hood.getHood());
  }

  @Test
  public void testGetAngle() {
    Angle currentAngle = hood.getAngle();
    assertNotNull(currentAngle);
    // Verify that the angle is not NaN
    assertFalse(Double.isNaN(currentAngle.in(edu.wpi.first.units.Units.Rotations)));
  }

  @Test
  public void testSetAngle() {
    Angle targetAngle = edu.wpi.first.units.Units.Rotations.of(0.25);
    Command setAngleCommand = hood.setAngle(targetAngle);
    
    assertNotNull(setAngleCommand);
    assertTrue(setAngleCommand instanceof Command);
    assertEquals(0.25, targetAngle.in(edu.wpi.first.units.Units.Rotations), 0.001);
  }

  @Test
  public void testSetAngleWithSupplier() {
    Angle targetAngle = edu.wpi.first.units.Units.Rotations.of(0.5);
    Command setAngleCommand = hood.setAngle(() -> targetAngle);
    
    assertNotNull(setAngleCommand);
    assertTrue(setAngleCommand instanceof Command);
    assertEquals(0.5, targetAngle.in(edu.wpi.first.units.Units.Rotations), 0.001);
  }

  @Test
  public void testGoHome() {
    Command homeCommand = hood.goHome();
    
    assertNotNull(homeCommand);
    assertTrue(homeCommand instanceof Command);
  }

  @Test
  public void testSetAngleSetpoint() {
    Angle targetAngle = edu.wpi.first.units.Units.Rotations.of(0.75);
    assertDoesNotThrow(() -> hood.setAngleSetpoint(targetAngle));
    assertEquals(0.75, targetAngle.in(edu.wpi.first.units.Units.Rotations), 0.001);
  }

  @Test
  public void testSetSpeed() {
    Dimensionless speed = edu.wpi.first.units.Units.Percent.of(50);
    Command speedCommand = hood.setSpeed(speed);
    
    assertNotNull(speedCommand);
    assertTrue(speedCommand instanceof Command);
    assertEquals(50, speed.in(edu.wpi.first.units.Units.Percent), 0.001);
  }

  @Test
  public void testAxisSpeed() {
    Dimensionless axisValue = edu.wpi.first.units.Units.Percent.of(75);
    Command axisCommand = hood.axisSpeed(() -> axisValue);
    
    assertNotNull(axisCommand);
    assertTrue(axisCommand instanceof Command);
    assertEquals(75, axisValue.in(edu.wpi.first.units.Units.Percent), 0.001);
  }

  @Test
  public void testAxisSpeedZero() {
    Dimensionless zeroAxis = edu.wpi.first.units.Units.Percent.of(0);
    Command axisCommand = hood.axisSpeed(() -> zeroAxis);
    
    assertNotNull(axisCommand);
    assertTrue(axisCommand instanceof Command);
    assertEquals(0, zeroAxis.in(edu.wpi.first.units.Units.Percent), 0.001);
  }

  @Test
  public void testStopCommand() {
    Command stopCmd = hood.stopCommand();
    
    assertNotNull(stopCmd);
    assertTrue(stopCmd instanceof Command);
  }

  @Test
  public void testStopMotor() {
    assertDoesNotThrow(() -> hood.stopMotor());
  }

  @Test
  public void testPeriodic() {
    assertDoesNotThrow(() -> hood.periodic());
  }

  @Test
  public void testSimulationPeriodic() {
    assertDoesNotThrow(() -> hood.simulationPeriodic());
  }

  @Test
  public void testMultipleAngleSetpoints() {
    Angle angle1 = edu.wpi.first.units.Units.Rotations.of(0.25);
    Angle angle2 = edu.wpi.first.units.Units.Rotations.of(0.5);
    Angle angle3 = edu.wpi.first.units.Units.Rotations.of(0.75);
    
    assertDoesNotThrow(() -> {
      hood.setAngleSetpoint(angle1);
      hood.setAngleSetpoint(angle2);
      hood.setAngleSetpoint(angle3);
    });
    
    // Verify all angles were created correctly
    assertEquals(0.25, angle1.in(edu.wpi.first.units.Units.Rotations), 0.001);
    assertEquals(0.5, angle2.in(edu.wpi.first.units.Units.Rotations), 0.001);
    assertEquals(0.75, angle3.in(edu.wpi.first.units.Units.Rotations), 0.001);
  }

  @Test
  public void testSpeedRanges() {
    double[] speedValues = {0.0, 0.25, 0.5, 0.75, 1.0, -0.5, -1.0};
    
    for (double speed : speedValues) {
      Dimensionless speedDim = edu.wpi.first.units.Units.Percent.of(speed * 100);
      Command cmd = hood.setSpeed(speedDim);
      assertNotNull(cmd);
      assertEquals(speed * 100, speedDim.in(edu.wpi.first.units.Units.Percent), 0.001);
    }
  }

  @Test
  public void testAxisSpeedRanges() {
    double[] axisValues = {-1.0, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1.0};
    
    for (double value : axisValues) {
      Dimensionless axis = edu.wpi.first.units.Units.Percent.of(value * 100);
      Command cmd = hood.axisSpeed(() -> axis);
      assertNotNull(cmd);
      assertEquals(value * 100, axis.in(edu.wpi.first.units.Units.Percent), 0.001);
    }
  }

  @Test
  public void testCommandSequencing() {
    Angle angle1 = edu.wpi.first.units.Units.Rotations.of(0.25);
    Angle angle2 = edu.wpi.first.units.Units.Rotations.of(0.75);

    Command cmd1 = hood.setAngle(angle1);
    Command cmd2 = hood.setAngle(angle2);

    assertNotNull(cmd1);
    assertNotNull(cmd2);
    assertEquals(0.25, angle1.in(edu.wpi.first.units.Units.Rotations), 0.001);
    assertEquals(0.75, angle2.in(edu.wpi.first.units.Units.Rotations), 0.001);
  }
}
