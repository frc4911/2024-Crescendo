// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.constants;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record ControlConstants(
    int driverControllerPort, int operatorControllerPort, double stickDeadband) {}
