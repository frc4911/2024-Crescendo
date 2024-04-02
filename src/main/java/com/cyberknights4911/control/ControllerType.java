// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.control;

import java.lang.annotation.Documented;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import javax.inject.Qualifier;

public enum ControllerType {
  Driver,
  Operator,
  None;

  @Qualifier
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @interface ControllerQualifier {
    ControllerType value();
  }
}
