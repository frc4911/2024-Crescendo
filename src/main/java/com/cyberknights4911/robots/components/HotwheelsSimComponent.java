// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.components;

import com.cyberknights4911.climb.modules.ClimbConstantsHotwheelsModule;
import com.cyberknights4911.climb.modules.ClimbIOSimModule;
import com.cyberknights4911.collect.modules.CollectConstantsSimModule;
import com.cyberknights4911.collect.modules.CollectIOSimModule;
import com.cyberknights4911.control.ControlConstantsModule;
import com.cyberknights4911.control.ControllerModule;
import com.cyberknights4911.drive.modules.DriveConstantsSimModule;
import com.cyberknights4911.drive.modules.DriveIOSimModule;
import com.cyberknights4911.drive.modules.DriveModule;
import com.cyberknights4911.indexer.modules.IndexerConstantsSimModule;
import com.cyberknights4911.indexer.modules.IndexerIOSimModule;
import com.cyberknights4911.robots.modules.RobotModule;
import com.cyberknights4911.robots.modules.SimbotModule;
import com.cyberknights4911.shooter.modules.ShooterConstantsSimModule;
import com.cyberknights4911.shooter.modules.ShooterIOSimModule;
import com.cyberknights4911.util.UtilModule;
import com.cyberknights4911.vision.modules.VisionConstantsSimModule;
import com.cyberknights4911.vision.modules.VisionIOSimModule;
import com.cyberknights4911.vision.modules.VisionModule;
import dagger.Component;

@Component(
    modules = {
      ClimbConstantsHotwheelsModule.class,
      ClimbIOSimModule.class,
      CollectConstantsSimModule.class,
      CollectIOSimModule.class,
      ControllerModule.class,
      ControlConstantsModule.class,
      DriveConstantsSimModule.class,
      DriveIOSimModule.class,
      DriveModule.class,
      IndexerConstantsSimModule.class,
      IndexerIOSimModule.class,
      RobotModule.class,
      ShooterConstantsSimModule.class,
      ShooterIOSimModule.class,
      SimbotModule.class,
      UtilModule.class,
      VisionConstantsSimModule.class,
      VisionIOSimModule.class,
      VisionModule.class,
    })
public interface HotwheelsSimComponent {}
