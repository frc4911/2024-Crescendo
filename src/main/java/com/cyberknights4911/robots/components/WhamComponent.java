// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.components;

import com.cyberknights4911.control.ControlConstantsModule;
import com.cyberknights4911.control.ControllerModule;
import com.cyberknights4911.drive.modules.DriveConstantsWhamModule;
import com.cyberknights4911.drive.modules.DriveIORealModule;
import com.cyberknights4911.drive.modules.DriveModule;
import com.cyberknights4911.robots.modules.RobotModule;
import com.cyberknights4911.robots.modules.WhamModule;
import com.cyberknights4911.slurpp.modules.SlurppIORealModule;
import com.cyberknights4911.util.UtilModule;
import com.cyberknights4911.vision.modules.VisionConstantsWhamModule;
import com.cyberknights4911.vision.modules.VisionIORealModule;
import com.cyberknights4911.vision.modules.VisionModule;
import dagger.Component;

@Component(
    modules = {
      ControllerModule.class,
      ControlConstantsModule.class,
      DriveConstantsWhamModule.class,
      DriveIORealModule.class,
      DriveModule.class,
      RobotModule.class,
      SlurppIORealModule.class,
      UtilModule.class,
      VisionConstantsWhamModule.class,
      VisionIORealModule.class,
      VisionModule.class,
      WhamModule.class,
    })
public interface WhamComponent {}
