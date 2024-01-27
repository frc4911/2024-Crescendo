// Copyright (c) 2023 FRC 4911
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
    int motorId1,
    int motorId2,
    double gearRatio,
    double fastVelocityRpm,
    double mediumVelocityRpm,
    double slowVelocityRpm,
    double errorVelocityRpm,
    PidValues feedBackValues,
    FeedForwardValues feedForwardValues) {}
