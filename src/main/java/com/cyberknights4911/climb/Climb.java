// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import javax.inject.Inject;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD");
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

  // Measured in OnShape
  // climber distance from front: 10.118
  // TODO: figure out base height (y for mount point)
  private static final Translation2d MOUNT_POINT = new Translation2d(10.118, 3.0);
  // TODO: figure out the fixed height as well as the segment start and end height
  private static final double FIXED_HEIGHT = 20.0;
  private static final double LENGTH_START = 0;
  // TODO: figure out this value
  private static final double WINCH_RADIUS = 1.0;

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final Mechanism2d mechanism;
  private final MechanismLigament2d leftSegment;
  private final MechanismLigament2d rightSegment;
  private final SysIdRoutine sysId;
  private final double winchConstant;

  @Inject
  public Climb(ClimbConstants constants, ClimbIO climbIO) {
    super();
    this.climbIO = climbIO;
    winchConstant = constants.gearRatio() * WINCH_RADIUS;
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    forwardLimit.initDefault(constants.forwardLimit());
    backwardLimit.initDefault(constants.backwardLimit());
    extendPosition.initDefault(constants.extendPosition());
    retractPosition.initDefault(constants.retractPosition());
    lockToggleTime.initDefault(constants.lockToggleTime());
    climbIO.configurePID(kP.get(), 0.0, kD.get());

    mechanism = new Mechanism2d(28.0, 28.0);
    // Where the climber is attached (These aren't really offset by 2 inches, but we want to see
    // them both in 2d)
    MechanismRoot2d rootLeft =
        mechanism.getRoot("climberLeft", MOUNT_POINT.getX() - 1.0, MOUNT_POINT.getY());
    MechanismRoot2d rootRight =
        mechanism.getRoot("climberRight", MOUNT_POINT.getX() + 1.0, MOUNT_POINT.getY());
    // The first segment is always at a fixed height, but the second segment "grows".
    leftSegment =
        rootLeft
            .append(new MechanismLigament2d("segmentLeft1", FIXED_HEIGHT, 90))
            .append(new MechanismLigament2d("segmentLeft2", LENGTH_START, 90));
    rightSegment =
        rootRight
            .append(new MechanismLigament2d("segmentRight1", FIXED_HEIGHT, 90))
            .append(new MechanismLigament2d("segmentRight2", LENGTH_START, 90));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climb/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    climbIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      climbIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (forwardLimit.hasChanged(hashCode()) || backwardLimit.hasChanged(hashCode())) {
      climbIO.configureLimits(forwardLimit.get(), backwardLimit.get());
    }

    leftSegment.setLength(LENGTH_START + winchConstant * inputs.leftPositionRad);
    rightSegment.setLength(LENGTH_START + winchConstant * inputs.rightPositionRad);

    Logger.recordOutput("Climb/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Stops the climber. */
  public void stop() {
    climbIO.stop();
  }

  boolean isClimbComplete() {
    // TODO: Check both left and right climbers against "retractPosition"
    return false;
  }

  void setClimbMode(Mode mode) {
    switch (mode) {
      case Right:
        // hold left, climb right
        break;
      case Left:
        // hold right, climb left
        break;
      case Both:
        // climb left and right
        break;
      case Hold:
        // hold left and right
        break;
      default:
        break;
    }
  }

  private Command setClimbLock(boolean locked) {
    return Commands.runOnce(
            () -> {
              setClimbLock(locked);
            },
            this)
        .andThen(Commands.waitSeconds(lockToggleTime.get()));
  }

  /** Extends both climbers to extended height. */
  public Command extendClimber() {
    return setClimbLock(false)
        .andThen(
            Commands.runOnce(() -> climbIO.setPositionLeft(extendPosition.get()))
                .until(() -> inputs.leftPositionRad >= extendPosition.get())
                .alongWith(
                    Commands.runOnce(() -> climbIO.setPositionRight(extendPosition.get()))
                        .until(() -> inputs.rightPositionRad >= extendPosition.get())));
  }

  /** Perform a climb. */
  public Command climb(Drive drive) {
    return new ClimbCommand(drive, this).andThen(setClimbLock(true));
  }

  public static enum Mode {
    Left,
    Right,
    Both,
    Hold
  }
}
