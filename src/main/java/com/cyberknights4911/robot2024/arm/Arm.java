// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber forwardLimit =
      new LoggedTunableNumber("Arm/forwardLimit");
  private static final LoggedTunableNumber backwardLimit =
      new LoggedTunableNumber("Arm/backwardLimit");

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Mechanism2d mechanism = new Mechanism2d(3, 3);
  private ArmFeedforward feedforward;
  private final SysIdRoutine sysId;

  public Arm(ArmConstants constants, ArmIO armIO) {
    super();
    this.armIO = armIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    forwardLimit.initDefault(constants.forwardLimit());
    backwardLimit.initDefault(constants.backwardLimit());
    feedforward = new ArmFeedforward(0, 0, 0);
    armIO.configurePID(kP.get(), kG.get(), kD.get());

    setupArmMechanism(mechanism);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    armIO.setVoltage(volts);
  }

  public void setCollectorAngle(CollectorAngle angle) {
    if (angle == CollectorAngle.EIGHTY) {
      armIO.setSolenoidState(true);
    } else if (angle == CollectorAngle.FORTY_FIVE) {
      armIO.setSolenoidState(false);
    }
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // TODO: Update the mechanism based on positionRad and solenoidState
    Logger.recordOutput("Arm/Mechanism", mechanism);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      armIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get());
    }
    if (forwardLimit.hasChanged(hashCode()) || backwardLimit.hasChanged(hashCode())) {
      armIO.configureLimits(forwardLimit.get(), backwardLimit.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Stops the arm. */
  public void stop() {
    armIO.stop();
  }

  private void setupArmMechanism(Mechanism2d mechanism2) {
    // Where the arm is attached
    MechanismRoot2d root = mechanism.getRoot("arm", 1, .5);
    // The first arm segment. This points downward in the initial configuration.
    MechanismLigament2d segment1 = root.append(new MechanismLigament2d("segment1", .25, 270));
    // The second arm segment. This is ALWAYS at a fixed angle relative to the first segment.
    MechanismLigament2d segment2 = segment1.append(new MechanismLigament2d("segment2", 1.5, 180));
    // The shooter. It's attached to end of arm and toggles between two fixed angles
    segment2.append(new MechanismLigament2d("shooter", 1, 225));
  }

  public Command stowArm() {
    return Commands.runOnce(
            () -> {
              // replace this with a call to move the arm to the stowed position.
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  // replace with an expression that only returns true when the arm is in the right
                  // position
                  return false;
                }));
  }

  public Command aimAmp() {
    return Commands.runOnce(
            () -> {
              // replace this with a call to move the arm to the "score amp" position.
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  // replace with an expression that only returns true when the arm is in the right
                  // position
                  return false;
                }));
  }

  public Command aimSubwoofer() {
    return Commands.runOnce(
            () -> {
              // replace this with a call to move the arm to the "score subwoofer" position.
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  // replace with an expression that only returns true when the arm is in the right
                  // position
                  return false;
                }));
  }

  public Command aimPodium() {
    return Commands.runOnce(
            () -> {
              // replace this with a call to move the arm to the "score podium" position.
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  // replace with an expression that only returns true when the arm is in the right
                  // position
                  return false;
                }));
  }

  /**
   * Automatically sets the aiming angle for an arbitrary distance from the speaker
   *
   * @param distanceMeters the horizontal distance from the speaker opening (not diagonal)
   */
  public Command aimForDistance(double distanceMeters) {
    return Commands.runOnce(
            () -> {
              // replace this with a call to move the arm to the optimal angle for the given
              // distance.
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  // replace with an expression that only returns true when the arm is in the right
                  // position
                  return false;
                }));
  }

  /**
   * Automatically and continuously sets the aiming angle for an arbitrary distance from the
   * speaker. Note: this command never finishes on its own, so don't wait for it.
   *
   * @param distanceSupplier returns the current horizontal distance from the speaker opening (not
   *     diagonal)
   */
  public Command aimContinuous(DoubleSupplier distanceSupplier) {
    return new AimContinuousCommand(this, distanceSupplier);
  }
}
