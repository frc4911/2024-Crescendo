// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for a collection of {@link Trigger}s. Provides some convenience when dealing with
 * binding multiple triggers to the same command.
 */
public final class Triggers {

  private final Trigger[] triggers;

  public Triggers(Trigger... triggers) {
    this.triggers = triggers;
  }

  /**
   * @see {@link Trigger#and(java.util.function.BooleanSupplier)}
   */
  public Triggers and(Triggers otherTriggers) {
    Trigger[] andTriggers = new Trigger[triggers.length * otherTriggers.triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      for (int j = 0; j < otherTriggers.triggers.length; j++) {
        andTriggers[i + j * triggers.length] = triggers[i].and(otherTriggers.triggers[j]);
      }
    }
    return new Triggers(andTriggers);
  }

  /**
   * @see {@link Trigger#debounce(double)}
   */
  public Triggers debounce(double seconds) {
    Trigger[] debouncedTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      debouncedTriggers[i] = triggers[i].debounce(seconds);
    }
    return new Triggers(debouncedTriggers);
  }

  /**
   * @see {@link Trigger#debounce(double, Debouncer.DebounceType)}
   */
  public Triggers debounce(double seconds, Debouncer.DebounceType type) {
    Trigger[] debouncedTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      debouncedTriggers[i] = triggers[i].debounce(seconds, type);
    }
    return new Triggers(debouncedTriggers);
  }

  /**
   * @see {@link Trigger#getAsBooleans()}
   */
  public boolean[] getAsBooleans() {
    boolean[] booleans = new boolean[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      booleans[i] = triggers[i].getAsBoolean();
    }
    return booleans;
  }

  /**
   * @see {@link Trigger#negate()}
   */
  public Triggers negate() {
    Trigger[] negatedTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      negatedTriggers[i] = triggers[i].negate();
    }
    return new Triggers(negatedTriggers);
  }

  /**
   * @see {@link Trigger#onFalse(Command)}
   */
  public Triggers onFalse(Command command) {
    Trigger[] onFalseTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      onFalseTriggers[i] = triggers[i].onFalse(command);
    }
    return new Triggers(onFalseTriggers);
  }

  /**
   * @see {@link Trigger#onTrue(Command)}
   */
  public Triggers onTrue(Command command) {
    Trigger[] onTrueTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      onTrueTriggers[i] = triggers[i].onTrue(command);
    }
    return new Triggers(onTrueTriggers);
  }

  /**
   * @see {@link Trigger#or(java.util.function.BooleanSupplier)}
   */
  public Triggers or(Triggers otherTriggers) {
    Trigger[] orTriggers = new Trigger[triggers.length * otherTriggers.triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      for (int j = 0; j < otherTriggers.triggers.length; j++) {
        orTriggers[i + j * triggers.length] = triggers[i].or(otherTriggers.triggers[j]);
      }
    }
    return new Triggers(orTriggers);
  }

  /**
   * @see {@link Trigger#toggleOnFalse(Command)}
   */
  public Triggers toggleOnFalse(Command command) {
    Trigger[] toggleOnFalseTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      toggleOnFalseTriggers[i] = triggers[i].toggleOnFalse(command);
    }
    return new Triggers(toggleOnFalseTriggers);
  }

  /**
   * @see {@link Trigger#toggleOnTrue(Command)}
   */
  public Triggers toggleOnTrue(Command command) {
    Trigger[] toggleOnTrueTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      toggleOnTrueTriggers[i] = triggers[i].toggleOnTrue(command);
    }
    return new Triggers(toggleOnTrueTriggers);
  }

  /**
   * @see {@link Trigger#whileFalse(Command)}
   */
  public Triggers whileFalse(Command command) {
    Trigger[] whileFalseTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      whileFalseTriggers[i] = triggers[i].whileFalse(command);
    }
    return new Triggers(whileFalseTriggers);
  }

  /**
   * @see {@link Trigger#whileTrue(Command)}
   */
  public Triggers whileTrue(Command command) {
    Trigger[] whileTrueTriggers = new Trigger[triggers.length];
    for (int i = 0; i < triggers.length; i++) {
      whileTrueTriggers[i] = triggers[i].whileTrue(command);
    }
    return new Triggers(whileTrueTriggers);
  }
}
