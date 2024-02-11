// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.advanced;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private List<DoubleSupplier> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  private SparkMaxOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / Module.ODOMETRY_FREQUENCY);
    }
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayDeque<>(100);
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayDeque<>(100);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    Drive.odometryLock.lock();
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
