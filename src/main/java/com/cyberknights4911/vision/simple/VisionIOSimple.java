// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.simple;

import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionIO;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public final class VisionIOSimple implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  public VisionIOSimple(VisionConstants visionConstants, CameraConstants constants) {
    camera = new PhotonCamera(constants.photonCameraName());
    camera.setDriverMode(false);

    // TODO(rbrewer) test roborio version of multi tag
    photonPoseEstimator =
        new PhotonPoseEstimator(
            visionConstants.layout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            constants.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isOnline = camera.isConnected();
    if (inputs.isOnline) {
      PhotonPipelineResult newResult = camera.getLatestResult();
      double latestTimestamp = newResult.getTimestampSeconds();
      if (Math.abs(latestTimestamp - inputs.lastTimestamp) > 1e-5) {
        inputs.lastResult = newResult;
        inputs.lastTimestamp = latestTimestamp;
      }
    }
  }

  @Override
  public void setReferencePose(Pose2d referencePose) {
    photonPoseEstimator.setReferencePose(referencePose);
  }

  @Override
  public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
    return photonPoseEstimator.update(cameraResult);
  }
}
