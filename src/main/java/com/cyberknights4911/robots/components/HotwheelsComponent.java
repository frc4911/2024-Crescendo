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
import com.cyberknights4911.drive.modules.DriveConstantsHotwheelsModule;
import com.cyberknights4911.drive.modules.DriveIORealModule;
import com.cyberknights4911.drive.modules.DriveModule;
import com.cyberknights4911.entrypoint.CyberKnightsRobot;
import com.cyberknights4911.indexer.modules.IndexerConstantsHotwheelsModule;
import com.cyberknights4911.indexer.modules.IndexerIORealModule;
import com.cyberknights4911.robots.modules.HotwheelsModule;
import com.cyberknights4911.robots.modules.RobotModule;
import com.cyberknights4911.shooter.modules.ShooterConstantsHotwheelsModule;
import com.cyberknights4911.shooter.modules.ShooterIORealModule;
import com.cyberknights4911.vision.modules.VisionConstantsHotwheelsModule;
import com.cyberknights4911.vision.modules.VisionIORealModule;
import com.cyberknights4911.vision.modules.VisionModule;
import dagger.Component;
import javax.inject.Singleton;

@Component(
    modules = {
      ClimbConstantsHotwheelsModule.class,
      ClimbIOStubModule.class,
      CollectConstantsHotwheelsModule.class,
      CollectIOStubModule.class,
      ControlConstantsModule.class,
      DriveConstantsHotwheelsModule.class,
      DriveIORealModule.class,
      DriveModule.class,
      HotwheelsModule.class,
      IndexerConstantsHotwheelsModule.class,
      IndexerIORealModule.class,
      RobotModule.class,
      ShooterConstantsHotwheelsModule.class,
      ShooterIORealModule.class,
      VisionConstantsHotwheelsModule.class,
      VisionIORealModule.class,
      VisionModule.class,
    })
@Singleton
public interface HotwheelsComponent {
  public CyberKnightsRobot robot();
}
