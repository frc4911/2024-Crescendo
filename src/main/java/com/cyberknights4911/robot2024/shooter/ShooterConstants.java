// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ShooterConstants(
    int shooterMotorTopId,
    int shooterMotorBottomId,
    int aimerMotorId,
    int guideMotorId,
    int sensorId,
    double beamThreshold,
    double aimerGearRatio,
    double aimerForwardLimit,
    double aimerBackwardLimit,
    double feedTime,
    double aimTime,
    double guidePercentOutput,
    double guideReversePercentOutput,
    double firePercentOutput,
    double collectPositionDegrees,
    double ampPositionDegrees,
    double trapPositionDegrees,
    double speakerPositionDegrees,
    double podiumPositionDegrees,
    PidValues shooterFeedBackValues,
    FeedForwardValues shooterFeedForwardValues,
    PidValues aimerFeedBackValues,
    FeedForwardValues aimerFeedForwardValues,
    PidValues guideFeedBackValues,
    FeedForwardValues guideFeedForwardValues) {}
