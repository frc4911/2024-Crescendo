// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV");

  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward;

  public Arm(ArmConstants constants, ArmIO armIO) {
    super();
    this.armIO = armIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedforward = new ArmFeedforward(0, 0, 0);
    armIO.configurePID(kP.get(), kG.get(), kD.get());
  }

  public void setVoltage(double volts) {
    armIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      armIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Stops the arm. */
  public void stop() {
    armIO.stop();
  }
}
