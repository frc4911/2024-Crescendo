// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.simple;

import com.cyberknights4911.logging.LoggedTunableNumber;
import com.cyberknights4911.vision.CameraConfig;
import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionIOInputsAutoLogged;
import com.cyberknights4911.vision.VisionUpdate;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public final class VisionSimple extends SubsystemBase {
  private static final Matrix<N3, N1> SINGLE_TAGS_STD_DEVS = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  private static final LoggedTunableNumber twoTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/twoTarget", 0.6);
  private static final LoggedTunableNumber threeTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/threeTarget", 0.4);
  private static final LoggedTunableNumber fourTargetDeviation =
      new LoggedTunableNumber("Vision/deviations/fourTarget", 0.2);

  private final Consumer<VisionUpdate> visionConsumer;
  private final CameraConfig[] cameraConfigs;

  private boolean updatePoseWithVisionReadings = true;
  private boolean useMaxValidDistanceAway = true;

  private final VisionConstants visionConstants;

  private final HashMap<String, Double> lastTimestamp = new HashMap<>();

  public VisionSimple(
      VisionConstants visionConstants,
      Consumer<VisionUpdate> visionConsumer,
      CameraConstants... cameraConstants) {
    super();
    this.visionConstants = visionConstants;
    this.visionConsumer = visionConsumer;

    Logger.recordOutput("Vision/updatePoseWithVisionReadings", updatePoseWithVisionReadings);
    Logger.recordOutput("Vision/useMaxValidDistanceAway", useMaxValidDistanceAway);

    for (AprilTag tag : visionConstants.layout().getTags()) {
      Logger.recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }

    cameraConfigs = new CameraConfig[cameraConstants.length];
    for (int i = 0; i < cameraConstants.length; i++) {
      cameraConfigs[i] =
          new CameraConfig(
              cameraConstants[i],
              new VisionIOSimple(visionConstants, cameraConstants[i]),
              new VisionIOInputsAutoLogged());
      Logger.recordOutput(
          "Vision/" + cameraConstants[i].name(),
          new Pose3d().transformBy(cameraConstants[i].robotToCamera()));
      lastTimestamp.put(cameraConstants[i].name(), 0.0);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameraConfigs.length; i++) {
      cameraConfigs[i].visionIO().updateInputs(cameraConfigs[i].inputs());
      Logger.processInputs(
          "Vision/" + cameraConfigs[i].constants().name(), cameraConfigs[i].inputs());
    }

    if (!updatePoseWithVisionReadings) {
      return;
    }

    for (int i = 0; i < cameraConfigs.length; i++) {
      updatePose(cameraConfigs[i]);
      Logger.recordOutput(
          "Vision/" + cameraConfigs[i].constants().name() + "Connected",
          cameraConfigs[i].inputs().lastTimestamp > 0.0);
    }
  }

  private void updatePose(CameraConfig cameraConfig) {
    // is this a new camera result?
    double prevTimestamp = lastTimestamp.get(cameraConfig.constants().name());
    if (prevTimestamp >= cameraConfig.inputs().lastTimestamp) {
      return;
    }
    lastTimestamp.put(cameraConfig.constants().name(), cameraConfig.inputs().lastTimestamp);

    cameraConfig
        .visionIO()
        .update(cameraConfig.inputs().lastResult)
        .ifPresent(
            estimatedPose -> {
              var robotPose = estimatedPose.estimatedPose.toPose2d();
              var stdDevs = getEstimationStdDevs(robotPose, cameraConfig.inputs().lastResult);

              Logger.recordOutput(
                  "Vision/" + cameraConfig.constants().name() + "/RobotPose", robotPose);
              Logger.recordOutput(
                  "Vision/" + cameraConfig.constants().name() + "/3DRobotPose",
                  estimatedPose.estimatedPose);
              Logger.recordOutput(
                  "Vision/" + cameraConfig.constants().name() + "/stdDev", stdDevs.toString());
              visionConsumer.accept(
                  new VisionUpdate(cameraConfig.inputs().lastTimestamp, robotPose, stdDevs));
            });
  }

  private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
    var estStdDevs = SINGLE_TAGS_STD_DEVS;
    var targets = result.getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = visionConstants.layout().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = MULTI_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }
}
