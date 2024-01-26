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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV");

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;

  public Climb(ClimbConstants constants, ClimbIO climbIO) {
    super();
    this.climbIO = climbIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    climbIO.configurePID(kP.get(), 0.0, kD.get());
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

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Stops the climber. */
  public void stop() {
    climbIO.stop();
  }
}
