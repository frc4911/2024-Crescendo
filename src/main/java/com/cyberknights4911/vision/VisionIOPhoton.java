// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  public VisionIOPhoton(VisionConstants visionConstants, CameraConstants constants) {
    camera = new PhotonCamera(constants.photonCameraName());
    camera.setDriverMode(false);

    // TODO(rbrewer) test roborio version of multi tag
    photonPoseEstimator =
        new PhotonPoseEstimator(
            visionConstants.layout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            constants.robotToCamera());
    // TODO(rbrewer) test this
    // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    NetworkTableInstance tables = NetworkTableInstance.getDefault();

    /*
     * based on https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
     * and https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub =
        tables
            .getTable("/photonvision/" + constants.photonCameraName())
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    tables.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000.0);

          // TODO(rbrewer): fix this
          synchronized (VisionIOPhoton.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.isOnline = camera.isConnected();
    inputs.lastTimestamp = lastTimestamp;
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
