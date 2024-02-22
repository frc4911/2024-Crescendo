// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Collect extends SubsystemBase {
  private static final LoggedTunableNumber beamThreshold =
      new LoggedTunableNumber("Collect/BeamThreshold");
  private static final LoggedTunableNumber ejectSpeed =
      new LoggedTunableNumber("Collect/EjectVelocityRPM");
  private static final LoggedTunableNumber collectSpeed =
      new LoggedTunableNumber("Collect/CollectVelocityRPM");
  private static final LoggedTunableNumber collectKp = new LoggedTunableNumber("Collect/CollectKp");
  private static final LoggedTunableNumber collectKd = new LoggedTunableNumber("Collect/CollectKd");
  private static final LoggedTunableNumber ejectTime = new LoggedTunableNumber("Collect/EjectTime");

  // Measured in OnShape
  private static final Translation2d SEGMENT_1_START = new Translation2d(3.267, 15.613);
  private static final Translation2d SEGMENT_1_END = new Translation2d(13.81, 8.322);
  private static final Translation2d SEGMENT_2_START = new Translation2d(6.435, 12.094);
  private static final Translation2d SEGMENT_2_END = new Translation2d(7.1, 11.773);
  private static final double SEGMENT_1_START_ANGLE =
      Math.toDegrees(Math.atan(SEGMENT_1_START.getY() / SEGMENT_1_START.getX()));
  private static final double SEGMENT_1_END_ANGLE =
      Math.toDegrees(Math.atan(SEGMENT_1_END.getY() / SEGMENT_1_END.getX()));
  // Add the angles to get the relative angle
  private static final double SEGMENT_2_START_ANGLE =
      -Math.toDegrees(
          Math.atan(SEGMENT_1_START.getX() / SEGMENT_1_START.getY())
              + Math.atan(SEGMENT_2_START.getX() / SEGMENT_2_START.getY()));
  private static final double SEGMENT_2_END_ANGLE =
      -Math.toDegrees(
          Math.atan(SEGMENT_1_END.getX() / SEGMENT_1_END.getY())
              + Math.atan(SEGMENT_2_END.getX() / SEGMENT_2_END.getY()));

  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private final Mechanism2d mechanism;
  private final SysIdRoutine sysId;
  private final MechanismLigament2d segment1;
  private final MechanismLigament2d segment2;

  public Collect(CollectConstants constants, CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
    collectKp.initDefault(constants.collectFeedBackValues().kP());
    collectKd.initDefault(constants.collectFeedBackValues().kD());
    collectSpeed.initDefault(constants.collectSpeed());
    ejectSpeed.initDefault(constants.ejectSpeed());
    ejectTime.initDefault(constants.ejectTime());
    collectIO.configureCollectPID(collectKp.get(), 0.0, collectKd.get());

    mechanism = new Mechanism2d(28.0, 28.0);
    // Where the collector is attached to the frame
    MechanismRoot2d root = mechanism.getRoot("collector", 14.0, 4.66);
    // The first collector segment. This is the portion that is moved by the solenoid.
    segment1 = root.append(new MechanismLigament2d("segment1", 16.073, SEGMENT_1_START_ANGLE));
    // The second collector segment. This represents the section reaches the ground.
    segment2 = segment1.append(new MechanismLigament2d("segment2", 13.794, SEGMENT_2_START_ANGLE));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Collect/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCollectVolts(voltage.in(Volts)), null, this));
  }

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakVoltage > beamThreshold.get();
  }

  /** Run collector open loop at the specified voltage. */
  public void runCollectVolts(double volts) {
    collectIO.setCollectVoltage(volts);
  }

  /** Run collector closed loop at the specified velocity. */
  public void runCollectVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    collectIO.setCollectVelocity(velocityRadPerSec);

    Logger.recordOutput("Collect/SetpointRPM", velocityRpm);
  }

  public void extendCollecter() {
    collectIO.setCollecterPosition(true);
  }

  public void retractCollecter() {
    collectIO.setCollecterPosition(false);
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);

    if (collectKp.hasChanged(hashCode()) || collectKd.hasChanged(hashCode())) {
      collectIO.configureCollectPID(collectKp.get(), 0.0, collectKd.get());
    }

    if (inputs.leftSolenoid && inputs.rightSolenoid) {
      segment1.setAngle(SEGMENT_1_END_ANGLE);
      segment2.setAngle(SEGMENT_2_END_ANGLE);
    } else if (!inputs.leftSolenoid && !inputs.rightSolenoid) {
      segment1.setAngle(SEGMENT_1_START_ANGLE);
      segment2.setAngle(SEGMENT_2_START_ANGLE);
    } else {
      System.out.println("ERROR: collector solenoids are in different states.");
    }

    Logger.recordOutput("Collect/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      stopCollector();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Returns the current velocity in RPM. */
  public double getCollectVelocityRpm() {
    double velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(inputs.collectVelocityRadPerSec);
    Logger.recordOutput("Collect/VelocityRPM", velocityRpm);
    return velocityRpm;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    collectIO.setCollectVoltage(volts);
  }

  /** Returns the average velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.collectVelocityRadPerSec;
  }

  /** Stops the collector. */
  public void stopCollector() {
    collectIO.stopCollector();
  }

  /**
   * Creates a command that runs the collector at a desired speed. The collector will stay running
   * after this command ends
   */
  private Command collectAtSpeed(LoggedTunableNumber desiredSpeed) {
    return Commands.runOnce(
        () -> {
          runCollectVelocity(desiredSpeed.get());
        },
        this);
  }

  /**
   * Creates a command that runs the collector at the proper collector speed. The collector will
   * stay running after this command ends, but the provided callback will be notified when the
   * gamepiece reaches the collector's sensor.
   *
   * @param sensorTrippedCallback will notify when the gamepiece reaches the sensor. At this point,
   *     the gamepiece is secure in the robot and the driver should be able to move the robot freely
   *     while the gamepiece moves through the robot.
   */
  public Command collectAtTunableSpeed(Runnable sensorTrippedCallback) {
    return Commands.runOnce(() -> runCollectVelocity(collectSpeed.get()), this)
        .until(this::isBeamBreakBlocked)
        .andThen(() -> sensorTrippedCallback.run());
  }

  /**
   * Creates a command that runs the collector at the proper collector speed, but stops when the
   * gamepiece reaches the sensor.
   */
  public Command collectGamePieceAndStop() {
    return collectAtSpeed(collectSpeed)
        .until(this::isBeamBreakBlocked)
        .andThen(this::stopCollector, this);
  }

  /** Creates a command for ejecting gamepieces backwards, out of the collector. */
  public Command ejectGamePiece() {
    return collectAtSpeed(ejectSpeed)
        .andThen(Commands.waitSeconds(ejectTime.get()))
        .andThen(this::stopCollector, this);
  }
}
