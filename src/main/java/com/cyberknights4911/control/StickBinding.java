// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.control;

import java.util.function.DoubleSupplier;

/** An abstraction layer for controller stick bindings. */
public interface StickBinding {
  /** A dummy supplier that is always zero. Use this as a placeholder or a fallback. */
  DoubleSupplier ALWAYS_ZERO = () -> 0.0;

  /**
   * Given a desired stick action, return the actual stick input supplier.
   *
   * @param action the stick action that is being bound.
   * @return the stick input supplier for the given action.
   */
  DoubleSupplier supplierFor(StickAction action);
}
