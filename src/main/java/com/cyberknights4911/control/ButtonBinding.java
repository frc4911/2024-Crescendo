package com.cyberknights4911.control;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An abstraction layer for controller button bindings. 
 */
public interface ButtonBinding<E extends ButtonAction> {
    /**
     * A dummy trigger that is always false. Use this as a placeholder or a fallback.
     */
    public static final Trigger ALWAYS_FALSE = new Trigger(new BooleanSupplier() {
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
