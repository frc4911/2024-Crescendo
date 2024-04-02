// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect;

import com.cyberknights4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

/**
 * Constants related to the collector.
 *
 * @param motorCollectLeftId id of the collector motor
 * @param sensorId id of the beam break sensor
 * @param collectGearRatio total gear ratio of the motor input to the collector output
 * @param collectFeedBackValues values to use in the collector PID controller
 * @param feedForwardValues values to use in the collector feed-forward controller
 */
@RecordBuilder
public record CollectConstants(
    int motorCollectLeftId,
    int motorCollectRightId,
    int sensorId,
    int solenoidLeftForwardId,
    int solenoidLeftReverseId,
    int solenoidRightForwardId,
    int solenoidRightReverseId,
    double collectGearRatio,
    double ejectTime,
    double ejectSpeed,
    double collectPercent,
    PidValues collectFeedBackValues) {}
