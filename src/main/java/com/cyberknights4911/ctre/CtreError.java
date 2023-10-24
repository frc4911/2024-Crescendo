package com.cyberknights4911.ctre;

import com.ctre.phoenix.ErrorCode;

/** Helper for checking ErrorCodes from CTRE classes. */
public final class CtreError {
    private final int canTimeoutMs;

    public CtreError(int canTimeoutMs) {
        this.canTimeoutMs = canTimeoutMs;
    }

    public void checkError(ErrorCode errorCode) {
        if (errorCode == ErrorCode.OK) {
            return;
        }
        // TODO: find a better way to report important errors. See:
        // https://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/Alert.java
        System.out.printf("Error: %s%n", errorCode.name());
    }

    public int canTimeoutMs() {
        return canTimeoutMs;
    }
}
