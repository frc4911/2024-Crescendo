// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import com.cyberknights4911.robot2024.collect.Collect;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
  private static final LoggedTunableNumber feedTime = new LoggedTunableNumber("Shooter/feedTime");
  private static final LoggedTunableNumber shootFastSpeed =
      new LoggedTunableNumber("Shooter/FastVelocityRPM");
  private static final LoggedTunableNumber shootMediumSpeed =
      new LoggedTunableNumber("Shooter/MediumVelocityRPM");
  private static final LoggedTunableNumber shootSlowSpeed =
      new LoggedTunableNumber("Shooter/SlowVelocityRPM");
  private static final LoggedTunableNumber shooterRpmError =
      new LoggedTunableNumber("Shooter/ErrorRPM");

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;
  private final SysIdRoutine sysId;

  public Shooter(ShooterConstants constants, ShooterIO shooterIO) {
    super();
    this.shooterIO = shooterIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedTime.initDefault(constants.feedTime());
    shootFastSpeed.initDefault(constants.fastVelocityRpm());
    shootMediumSpeed.initDefault(constants.mediumVelocityRpm());
    shootSlowSpeed.initDefault(constants.slowVelocityRpm());
    shooterRpmError.initDefault(constants.errorVelocityRpm());
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    shooterIO.configurePID(kP.get(), 0.0, kD.get());

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
    shooterIO.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    shooterIO.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      shooterIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRpm() {
    double velocityRpm = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    Logger.recordOutput("Shooter/VelocityRPM", velocityRpm);
    return velocityRpm;
  }

  /** Stops the shooter. */
  public void stop() {
    shooterIO.stop();
  }

  /**
   * Creates a command that spins the shooter at the desired speed. This command will not end until
   * the actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  private Command spinAtSpeed(LoggedTunableNumber desiredSpeed) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runVelocity(desiredSpeed.get());
            },
            this),
        Commands.waitUntil(
            () -> {
              return Math.abs(
                      Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
                          - desiredSpeed.get())
                  < shooterRpmError.get();
            }));
  }

  /**
   * Creates a command that spins the shooter at the fast speed. This command will not end until the
   * actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  public Command spinFast() {
    return spinAtSpeed(shootFastSpeed);
  }

  /**
   * Creates a command that spins the shooter at the medium speed. This command will not end until
   * the actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  public Command spinMedium() {
    return spinAtSpeed(shootMediumSpeed);
  }

  /**
   * Creates a command that spins the shooter at the slow speed. This command will not end until the
   * actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  public Command spinSlow() {
    return spinAtSpeed(shootSlowSpeed);
  }

  /**
   * Creates a complete command for firing a note
   *
   * @param collect the collector subsystem
   */
  public Command fireNote(Collect collect) {
    // sequence:
    // 1. spin shooter
    // 2. feed gamepiece
    // 3. wait fixed delay
    // 4. stop shooter and collector

    return spinFast()
        .andThen(collect.feedGamePieceToShooter())
        .andThen(Commands.waitSeconds(feedTime.get()))
        .andThen(this::stop, this)
        .andThen(collect::stop, collect);
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
