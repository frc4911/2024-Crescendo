// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.cyberknights4911.drive.DriveConstants.ModuleConstants;
import dagger.assisted.AssistedFactory;

@AssistedFactory
public interface ModuleIOTalonFXFactory {
  public ModuleIOTalonFX create(ModuleConstants moduleConstants);
}