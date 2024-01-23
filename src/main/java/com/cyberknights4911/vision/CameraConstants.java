// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision;

import edu.wpi.first.math.geometry.Transform3d;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record CameraConstants(String name, String photonCameraName, Transform3d robotToCamera) {}
