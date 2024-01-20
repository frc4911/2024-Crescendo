// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.vision;

import org.photonvision.PhotonCamera;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;

  public VisionIOPhoton(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }
}
