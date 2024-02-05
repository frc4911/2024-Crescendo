// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.drive.OdometryThread;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public final class SparkOdometryThread implements OdometryThread {
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private final Lock odometryLock = new ReentrantLock();
  private final Notifier notifier;
  private final double odometryFrequency;

  public SparkOdometryThread(DriveConstants driveConstants) {
    this.odometryFrequency = driveConstants.odometryFrequency();
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkOdometryThread");
  }

  @Override
  public void lock() {
    odometryLock.lock();
  }

  @Override
  public void unlock() {
    odometryLock.unlock();
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / odometryFrequency);
    }
  }

  @Override
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayDeque<>(100);
    odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
    throw new IllegalStateException("Can't register Phoenix device on SparkOdometryThread");
  }

  @Override
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayDeque<>(100);
    odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    odometryLock.lock();
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      odometryLock.unlock();
    }
  }
}
