// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean isOnline = false;
    public double lastTimeStamp = 0;
    public PhotonPipelineResult lastResult = new PhotonPipelineResult();
  }

  public default void updateInputs(VisionIOInputsAutoLogged inputs) {}
}
