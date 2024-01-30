// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV");
  private static final LoggedTunableNumber lockToggleTime =
      new LoggedTunableNumber("Climb/LockToggleTime");
  private static final LoggedTunableNumber extendPosition =
      new LoggedTunableNumber("Climb/extendPosition");
  private static final LoggedTunableNumber retractPosition =
      new LoggedTunableNumber("Climb/retractPosition");
  private static final LoggedTunableNumber forwardLimit =
      new LoggedTunableNumber("Climb/forwardLimit");
  private static final LoggedTunableNumber backwardLimit =
      new LoggedTunableNumber("Climb/backwardLimit");

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final Mechanism2d mechanism = new Mechanism2d(3, 3);
  private SimpleMotorFeedforward feedforward;

  public Climb(ClimbConstants constants, ClimbIO climbIO) {
    super();
    this.climbIO = climbIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    forwardLimit.initDefault(constants.forwardLimit());
    backwardLimit.initDefault(constants.backwardLimit());
    extendPosition.initDefault(constants.extendPosition());
    retractPosition.initDefault(constants.retractPosition());
    lockToggleTime.initDefault(constants.lockToggleTime());
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    climbIO.configurePID(kP.get(), 0.0, kD.get());

    setupClimberMechanism(mechanism);
  }

  public void setClimbVoltage(double volts) {
    climbIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      climbIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    }
    if (forwardLimit.hasChanged(hashCode()) || backwardLimit.hasChanged(hashCode())) {
      climbIO.configureLimits(forwardLimit.get(), backwardLimit.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Stops the climber. */
  public void stop() {
    climbIO.stop();
  }

  private void setupClimberMechanism(Mechanism2d mechanism2) {
    // Where the climber is attached
    MechanismRoot2d root = mechanism.getRoot("climber", 1.5, 0);
    // The first climber segment. This is the portion that is always at a fixed height.
    MechanismLigament2d segment = root.append(new MechanismLigament2d("segment1", 1, 90));
    // The second climber segment. This represents the section that extends upward.
    segment.append(new MechanismLigament2d("segment2", 0, 90));
  }

  private Command setClimbLock(boolean locked) {
    return Commands.runOnce(
            () -> {
              setClimbLock(locked);
            },
            this)
        .andThen(Commands.waitSeconds(lockToggleTime.get()));
  }

  /** Extends the climbers to climbing height. */
  public Command extendClimber() {
    return setClimbLock(false)
        .andThen(
            Commands.runOnce(
                () -> {
                  climbIO.setPosition(extendPosition.get());
                },
                this))
        .until(
            () -> {
              return inputs.positionLinear >= extendPosition.get();
            });
  }

  /** Retracts the climbers to the climbed height. */
  public Command climb() {
    return Commands.runOnce(
            () -> {
              climbIO.setPosition(retractPosition.get());
            },
            this)
        .until(
            () -> {
              return inputs.positionLinear <= retractPosition.get();
            })
        .andThen(setClimbLock(true));
  }
}
