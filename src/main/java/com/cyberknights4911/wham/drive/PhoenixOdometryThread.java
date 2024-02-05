// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.drive.OdometryThread;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class PhoenixOdometryThread extends Thread implements OdometryThread {
  // Prevents conflicts when registering signals
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private boolean isCANFD = false;
  private final Lock odometryLock = new ReentrantLock();
  private final double odometryFrequency;

  public PhoenixOdometryThread(DriveConstants driveConstants) {
    this.odometryFrequency = driveConstants.odometryFrequency();
    setName("PhoenixOdometryThread");
    setDaemon(true);
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
      super.start();
    }
  }

  @Override
  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
    Queue<Double> queue = new ArrayDeque<>(100);
    signalsLock.lock();
    odometryLock.lock();
    try {
      isCANFD = CANBus.isNetworkFD(device.getNetwork());
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      signalsLock.unlock();
      odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    throw new IllegalStateException("Can't register DoubleSupplier on PhoenixOdometryThread");
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

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD) {
          BaseStatusSignal.waitForAll(2.0 / odometryFrequency, signals);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing. No reasoning for this behavior
          // is provided by the documentation.
          Thread.sleep((long) (1000.0 / odometryFrequency));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      odometryLock.lock();
      try {
        double timestamp = Logger.getRealTimestamp() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.length > 0) {
          timestamp -= totalLatency / signals.length;
        }
        for (int i = 0; i < signals.length; i++) {
          queues.get(i).offer(signals[i].getValueAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        odometryLock.unlock();
      }
    }
  }
}
