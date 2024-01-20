// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumSet;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera;
  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  public VisionIOPhoton(String cameraName) {
    camera = new PhotonCamera(cameraName);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    camera.setDriverMode(false);

    /*
     * based on https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
     * and https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub =
        inst.getTable("/photonvision/" + cameraName)
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();
          double timestamp = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000.0);
          synchronized (VisionIOPhoton.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  @Override
  public synchronized void updateInputs(VisionIOInputsAutoLogged inputs) {
    inputs.isOnline = camera.isConnected();
    inputs.lastResult = lastResult;
    inputs.lastTimeStamp = lastTimestamp;
  }
}
