// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.control;

import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.control.ButtonAction;
import com.cyberknights4911.control.ButtonBinding;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.control.StickBinding;
import com.cyberknights4911.control.Triggers;
import com.cyberknights4911.logging.Alert;
import com.cyberknights4911.logging.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public final class ControllerBinding implements StickBinding, ButtonBinding {
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  private final CommandXboxController driver;
  private final CommandXboxController operator;

  public ControllerBinding(ControlConstants constants) {
    driver = new CommandXboxController(constants.driverPort());
    operator = new CommandXboxController(constants.operatorPort());
  }

  @Override
  public DoubleSupplier supplierFor(StickAction action) {
    switch (action) {
      case FORWARD:
        return () -> -driver.getLeftY();
      case STRAFE:
        return () -> -driver.getLeftX();
      case ROTATE:
        return () -> -driver.getRightX();
      default:
        return ALWAYS_ZERO;
    }
  }

  @Override
  public Triggers triggersFor(ButtonAction action) {
    switch (action) {
      case ZeroGyro:
        return new Triggers(driver.y());
      case Brake:
        return new Triggers(driver.x());
        // case ZeroSpeaker:
        //   return new Triggers(driver.start());
        // case AmpLockOn:
        //   return new Triggers(driver.leftTrigger());
        // case SpeakerLockOn:
        //   return new Triggers(driver.rightTrigger());
      case CollectNote:
        return new Triggers(operator.a());
      case StowCollector:
        return new Triggers(operator.b());
      case AimPodium:
        return new Triggers(operator.povLeft());
      case AimSubwoofer:
        return new Triggers(operator.povRight());
      case FireNoteSpeaker:
        return new Triggers(operator.rightTrigger());
      case FireNoteAmp:
        return new Triggers(operator.leftTrigger());
      case FireNoteTrap:
        return new Triggers(operator.y());
      default:
        return new Triggers(ALWAYS_FALSE);
    }
  }

  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
  }

  public void setDriverRumble(boolean enabled) {
    driver.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }

  public void setOperatorRumble(boolean enabled) {
    operator.getHID().setRumble(RumbleType.kBothRumble, enabled ? 1 : 0);
  }
}
