// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean isOnline = false;
    public double lastTimestamp = 0;
    public PhotonPipelineResult lastResult = new PhotonPipelineResult();
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setReferencePose(Pose2d referencePose) {}

  public default Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
    return Optional.empty();
  }
}
