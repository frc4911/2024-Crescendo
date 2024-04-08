// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.components;

import com.cyberknights4911.climb.modules.ClimbConstantsHotwheelsModule;
import com.cyberknights4911.climb.modules.ClimbIOStubModule;
import com.cyberknights4911.collect.modules.CollectConstantsHotwheelsModule;
import com.cyberknights4911.collect.modules.CollectIOStubModule;
import com.cyberknights4911.control.ControlConstantsModule;
import com.cyberknights4911.control.ControllerModule;
import com.cyberknights4911.drive.modules.DriveConstantsHotwheelsModule;
import com.cyberknights4911.drive.modules.DriveIOStubModule;
import com.cyberknights4911.drive.modules.DriveModule;
import com.cyberknights4911.entrypoint.CyberKnightsRobot;
import com.cyberknights4911.indexer.modules.IndexerConstantsHotwheelsModule;
import com.cyberknights4911.indexer.modules.IndexerIOStubModule;
import com.cyberknights4911.robots.modules.HotwheelsModule;
import com.cyberknights4911.robots.modules.RobotModule;
import com.cyberknights4911.shooter.modules.ShooterConstantsHotwheelsModule;
import com.cyberknights4911.shooter.modules.ShooterIOStubModule;
import com.cyberknights4911.util.UtilModule;
import com.cyberknights4911.vision.modules.VisionConstantsHotwheelsModule;
import com.cyberknights4911.vision.modules.VisionIOStubModule;
import com.cyberknights4911.vision.modules.VisionModule;
import dagger.Component;
import javax.inject.Singleton;

@Component(
    modules = {
      ClimbConstantsHotwheelsModule.class,
      ClimbIOStubModule.class,
      CollectConstantsHotwheelsModule.class,
      CollectIOStubModule.class,
      ControllerModule.class,
      ControlConstantsModule.class,
      DriveConstantsHotwheelsModule.class,
      DriveIOStubModule.class,
      DriveModule.class,
      HotwheelsModule.class,
      IndexerConstantsHotwheelsModule.class,
      IndexerIOStubModule.class,
      RobotModule.class,
      ShooterConstantsHotwheelsModule.class,
      ShooterIOStubModule.class,
      UtilModule.class,
      VisionConstantsHotwheelsModule.class,
      VisionIOStubModule.class,
      VisionModule.class,
    })
@Singleton
public interface HotwheelsReplayComponent {
  public CyberKnightsRobot robot();
}
