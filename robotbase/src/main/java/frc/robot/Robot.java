// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.controller.RumbleDriverController;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveStop;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.CoDriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.controls.PitControls;
import frc.robot.controls.TuningControls;
import frc.robot.generated.TunerConstants;
import frc.robot.networkTables.AutoChooserManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRunner;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tunnel;
import frc.robot.triggers.ShiftWarning;
import frc.robot.vision.CameraManager;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  public static DriverControls driverControls;
  public static CoDriverControls coDriverControls;
  public static TuningControls tuningControls;
  public static PitControls pitControls;
  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;

  public static Alliance alliance = null;
  public static MatchType matchType = null;

  public static InitRobotCommand m_InitRobotCommand;

  private AutoChooserManager m_autoChooserManager;
  public static IntakeRunner intake;
  public static IntakePivot intakePivot;
  public static Shooter shooter;
  public static Hood hood;
  public static Floor floor;
  public static Feeder feeder;
  public static Kicker kicker;
  public static Tunnel tunnel;

  boolean lastDriveMode;

  public static CameraManager cameraManager;
  public static ShooterCurveManager shooterCurveManager;
  public static ShotCalculator shotCalculator;

  public static ShiftTimer shiftTimer;

  private static final Field2d m_robotField = new Field2d();
  private final Timer m_curveUpdateTimer = new Timer();

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );

  public Robot() {
    swerve = TunerConstants.createDrivetrain();

    intake = new IntakeRunner();
    intakePivot = new IntakePivot();
    shooter = new Shooter();
    hood = new Hood();
    floor = new Floor();
    feeder = new Feeder();
    kicker = new Kicker();
    tunnel = new Tunnel();

    cameraManager = new CameraManager();

    swerve.registerTelemetry(logger::telemeterize);
    shotCalculator = new ShotCalculator();
    shooterCurveManager = new ShooterCurveManager();

    driverControls = new DriverControls();
    coDriverControls = new CoDriverControls();
    m_defaultDrive = new DefaultDrive(
      driverControls::getLeftX,
      driverControls::getLeftY,
      driverControls::getRightX
    );

    swerve.registerTelemetry(logger::telemeterize);
    swerve.setDefaultCommand(m_defaultDrive);

    hood.setDefaultCommand(hood.goHome());
    shooter.setDefaultCommand(shooter.setIdleVelocity());

    m_autoChooserManager = new AutoChooserManager();
    m_InitRobotCommand = new InitRobotCommand();

    SmartDashboard.putBoolean("Drive Mode Brake", false);

    shiftTimer = new ShiftTimer();

    Trigger shiftWarning = new ShiftWarning().warn();
    /**
     * Making this trigger require being attached to the FMS to
     * avoid us getting annoyed with it rumbling
     *
     * Should remove the fms attachment requirement for drive practice
     */
    shiftWarning
      // .and(DriverStation::isFMSAttached)
      .onTrue(new RumbleDriverController());

    // DON'T DELETE - Load the april tag field
    // This prevents a loop overrun when we first access the constants
    AprilTagFieldLayout layout = Constants.FieldConstants.FIELD_LAYOUT;

    SmartDashboard.putData("Robot Field", m_robotField);

    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    StatusLogger.disableAutoLogging();

    // Starts recording to data log
    DataLogManager.logNetworkTables(true);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    DriverStation.silenceJoystickConnectionWarning(true);

    // Warmp choreo
    CommandScheduler.getInstance().schedule(
      Constants.CHOREO.AUTO_FACTORY.warmupCmd()
    );
  }

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().schedule(m_InitRobotCommand);
    m_curveUpdateTimer.start();
  }

  @Override
  public void robotPeriodic() {
    Robot.cameraManager.updateResult();
    Robot.cameraManager.addSwerveEstimates(Robot.swerve::addVisionMeasurement);
    Robot.shotCalculator.updateCalculatedShot();
    SmartDashboard.putBoolean(
      "fire control approval",
      shotCalculator.fireControlApproval().getAsBoolean()
    );
    SmartDashboard.putBoolean(
      "in alliance zone",
      shotCalculator.isInAllianceZone()
    );
    if (SmartDashboard.getBoolean("Drive Mode Brake", false) != lastDriveMode) {
      lastDriveMode = SmartDashboard.getBoolean("Drive Mode Brake", false);
      if (
        SmartDashboard.getBoolean("Drive Mode Brake", true)
      ) new DriveSetBrake();
    } else {
      new DriveSetCoast();
    }

    CommandScheduler.getInstance().run();

    m_robotField.setRobotPose(swerve.getFieldRelativePose2d());

    if (
      (matchType == MatchType.None || matchType == MatchType.Practice) &&
      m_curveUpdateTimer.advanceIfElapsed(
        Constants.SHOOTER.CURVE_UPDATE_INTERVAL.in(Units.Seconds)
      )
    ) {
      shooterCurveManager.updateCurveValues();
    }
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(new StopAllMotors());
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooserManager.getSelectedCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    pitControls = new PitControls();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  public static void initializeTuningController() {
    if (matchType == null) return;

    switch (matchType) {
      case Elimination:
      case Qualification:
        return;
      default:
        tuningControls = new TuningControls();
    }
  }
}
