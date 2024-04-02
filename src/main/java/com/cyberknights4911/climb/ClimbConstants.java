// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb;

import com.cyberknights4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ClimbConstants(
    int leftMotorId,
    int rightMotorId,
    int solenoidLeftId,
    int solenoidRightId,
    double forwardLimit,
    double backwardLimit,
    double extendPosition,
    double retractPosition,
    double gearRatio,
    double lockToggleTime,
    PidValues feedBackValues) {}
