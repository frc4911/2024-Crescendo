// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.lights;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Lights extends SubsystemBase {
  private SerialPort arduino;

  private Mode message = Mode.Red;

  public Lights() {
    try {
      //   arduino = new SerialPort(9600, SerialPort.Port.kUSB);
    } catch (Exception e) {
      System.out.println("Can't open port");
      e.printStackTrace();
    }
  }

  public void setMode(Mode mode) {
    this.message = mode;
  }

  @Override
  public void periodic() {
    if (arduino == null) {
      return;
    }
    arduino.write(new byte[] {}, 1);
  }

  public enum Mode {
    Red(0),
    Blue(240);

    final byte message;

    Mode(Integer message) {
      this.message = message.byteValue();
    }
  }
}
