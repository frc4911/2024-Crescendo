// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.control;

import com.cyberknights4911.control.ButtonAction.ButtonKey;
import com.cyberknights4911.control.ControllerType.ControllerQualifier;
import com.cyberknights4911.control.StickAction.StickKey;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

@Module
public interface ControllerModule {

  @Provides
  @ControllerQualifier(ControllerType.Driver)
  public static CommandXboxController providesDriver(ControlConstants constants) {
    return new CommandXboxController(constants.driverPort());
  }

  @Provides
  @ControllerQualifier(ControllerType.Operator)
  public static CommandXboxController providesOperator(ControlConstants constants) {
    return new CommandXboxController(constants.operatorPort());
  }

  @Provides
  @IntoMap
  @StickKey(StickAction.FORWARD)
  public static DoubleSupplier providesForward(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return () -> -driver.getLeftY();
  }

  @Provides
  @IntoMap
  @StickKey(StickAction.STRAFE)
  public static DoubleSupplier providesStrafe(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return () -> -driver.getLeftX();
  }

  @Provides
  @IntoMap
  @StickKey(StickAction.ROTATE)
  public static DoubleSupplier providesRotate(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return () -> -driver.getRightX();
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.ZeroSpeaker)
  public static Triggers providesZeroSpeaker(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return new Triggers(driver.start());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.ZeroGyro)
  public static Triggers providesZeroGyro(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return new Triggers(driver.y());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.Brake)
  public static Triggers providesBrake(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return new Triggers(driver.x());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.SpeakerLockOn)
  public static Triggers providesSpeakerLockOn(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return new Triggers(driver.rightTrigger());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.AmpLockOn)
  public static Triggers providesAmpLockOn(
      @ControllerQualifier(ControllerType.Driver) CommandXboxController driver) {
    return new Triggers(driver.leftTrigger());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.CollectNote)
  public static Triggers providesCollectNote(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.a());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.ReverseCollect)
  public static Triggers providesReverseCollect(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.x());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.StowCollector)
  public static Triggers providesStowCollector(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.b());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.AimPodium)
  public static Triggers providesAimPodium(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.povLeft());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.AimSubwoofer)
  public static Triggers providesAimSubwoofer(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.povRight());
  }

  @Provides
  @IntoMap
  @ButtonKey(ButtonAction.FireNoteSpeaker)
  public static Triggers providesFireNoteSpeaker(
      @ControllerQualifier(ControllerType.Operator) CommandXboxController operator) {
    return new Triggers(operator.rightTrigger());
  }
}
