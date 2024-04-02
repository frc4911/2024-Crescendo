// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import com.cyberknights4911.logging.LoggedTunableNumberFactory;
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
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public class Collect extends SubsystemBase {
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

  private final LoggedTunableNumber collectOutput;
  private final LoggedTunableNumber collectKp;
  private final LoggedTunableNumber collectKd;
  private final LoggedTunableNumber ejectTime;
  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private final Mechanism2d mechanism;
  private final SysIdRoutine sysId;
  private final MechanismLigament2d segment1;
  private final MechanismLigament2d segment2;

  @Inject
  public Collect(
      CollectConstants constants, CollectIO collectIO, LoggedTunableNumberFactory numberFactory) {
    super();
    this.collectIO = collectIO;
    collectKp =
        numberFactory.getNumber("Collect/CollectKp", constants.collectFeedBackValues().kP());
    collectKd =
        numberFactory.getNumber("Collect/CollectKd", constants.collectFeedBackValues().kD());
    ejectTime = numberFactory.getNumber("Collect/EjectTime", constants.ejectTime());
    collectOutput = numberFactory.getNumber("Collect/OutputPercent", constants.collectPercent());

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

    // Logger.recordOutput("Collect/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      collectIO.stopCollector();
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

  public Command ejectNote() {
    return Commands.runOnce(
        () -> {
          collectIO.setCollecterPosition(true);
          collectIO.setCollectOutput(-collectOutput.get());
        },
        this);
  }

  public Command extendCollecter() {
    return Commands.runOnce(
        () -> {
          collectIO.setCollecterPosition(true);
          collectIO.setCollectOutput(collectOutput.get());
        },
        this);
  }

  public Command retractCollecter() {
    return Commands.runOnce(
        () -> {
          collectIO.stopCollector();
          collectIO.setCollecterPosition(false);
        },
        this);
  }
}
