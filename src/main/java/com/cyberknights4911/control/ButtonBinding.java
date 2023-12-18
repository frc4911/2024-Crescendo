// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.control;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** An abstraction layer for controller button bindings. */
public interface ButtonBinding<E extends ButtonAction> {
  /** A dummy trigger that is always false. Use this as a placeholder or a fallback. */
  Trigger ALWAYS_FALSE =
      new Trigger(
          new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
              return false;
            }
          });

  /**
   * Given a desired button action, return the actual button triggers.
   *
   * @param action the button action that is being bound.
   * @return the input triggers that maps to the given action.
   */
  Triggers triggersFor(E action);
}
