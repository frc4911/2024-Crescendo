// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public interface OdometryThread {

  public void lock();

  public void unlock();

  public void start();

  public Queue<Double> makeTimestampQueue();

  public Queue<Double> registerSignal(DoubleSupplier signal);

  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal);
}
