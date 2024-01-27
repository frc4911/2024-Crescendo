// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ClimbConstants(
    int motorId1,
    int motorId2, // Is there more than one motor for climb?
    double forwardLimit,
    double backwardLimit,
    double gearRatio,
    PidValues feedBackValues,
    FeedForwardValues feedForwardValues) {}
