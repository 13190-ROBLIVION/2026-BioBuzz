package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A Gamepad subclass that adds an "analog" bumper on top of FTC's digital ones.
 *
 * The driver station sends bumpers as plain booleans - unlike the triggers,
 * there is no pressure value to read, which is why "right bumper = power"
 * can't work the obvious way. This class emulates the missing axis: hold the
 * bumper and a 0..1 value ramps up over real time, release it and the value
 * drops back to 0. A short press is reported as a tap, so one button can do
 * coarse steps (taps) and fine ramps (holds).
 *
 * Call update() once per tick with the real gamepad and delta time, then read
 * it like any other gamepad:
 *
 *   shooterPad.update(gamepad1, deltaTimeSeconds);
 *
 *   if (shooterPad.right_bumper_tapped) { ... }
 *   double finesse = shooterPad.right_bumper_analog;
 *
 * Edge flags (just_pressed / just_released / tapped / hold_started) are true
 * for exactly the one update() in which the transition happened. Field names
 * stay snake_case on purpose to match the Gamepad fields they extend.
 */
public class AnalogBumperGamepad extends Gamepad {
    // ----- Emulated analog values (0..1 while held past the threshold, 0 otherwise) -----
    public float right_bumper_analog;
    public float left_bumper_analog;

    // ----- Per-tick edge flags, valid until the next update() -----
    public boolean right_bumper_just_pressed;
    public boolean right_bumper_just_released;
    public boolean right_bumper_tapped; // released before the tap threshold
    public boolean right_bumper_hold_started; // crossed the tap threshold this tick
    public boolean right_bumper_holding; // pressed and past the tap threshold

    public boolean left_bumper_just_pressed;
    public boolean left_bumper_just_released;
    public boolean left_bumper_tapped;
    public boolean left_bumper_hold_started;
    public boolean left_bumper_holding;

    // ----- Tuning -----
    // Holds shorter than this are taps; longer ones start ramping the analog value.
    public double tapThresholdSeconds = 0.30;
    // How many seconds of holding take the analog value from 0 to 1.
    public double rampSeconds = 0.60;

    // ----- Per-bumper bookkeeping (written by update, read by nobody else) -----
    private static class BumperState {
        boolean pressedLast;
        boolean wasHolding;
        double heldSeconds;
        boolean justPressed;
        boolean justReleased;
        boolean tapped;
        boolean holdStarted;
        boolean holding;
        float analog;
    }

    private final BumperState rightState = new BumperState();
    private final BumperState leftState = new BumperState();

    /**
     * Pull in the real gamepad's state and age it by deltaTimeSeconds.
     * Call this every tick, before reading anything off this class.
     *
     * @param source the real gamepad (gamepad1 / gamepad2) to mirror
     * @param deltaTimeSeconds seconds since the previous tick
     */
    public void update(Gamepad source, double deltaTimeSeconds) {
        copy(source); // bring every standard field (buttons, sticks, bumpers...) up to date

        // Clamp delta so a loop hiccup or a DS reconnect can't teleport the ramp.
        double deltaTime = Math.max(0.0, Math.min(deltaTimeSeconds, 0.1));

        updateBumper(rightState, right_bumper, deltaTime);
        right_bumper_analog = rightState.analog;
        right_bumper_just_pressed = rightState.justPressed;
        right_bumper_just_released = rightState.justReleased;
        right_bumper_tapped = rightState.tapped;
        right_bumper_hold_started = rightState.holdStarted;
        right_bumper_holding = rightState.holding;

        updateBumper(leftState, left_bumper, deltaTime);
        left_bumper_analog = leftState.analog;
        left_bumper_just_pressed = leftState.justPressed;
        left_bumper_just_released = leftState.justReleased;
        left_bumper_tapped = leftState.tapped;
        left_bumper_hold_started = leftState.holdStarted;
        left_bumper_holding = leftState.holding;
    }

    private void updateBumper(BumperState bumper, boolean pressed, double deltaTimeSeconds) {
        bumper.justPressed = pressed && !bumper.pressedLast;
        bumper.justReleased = !pressed && bumper.pressedLast;

        // A release counts as a tap when the bumper never made it past the
        // threshold - heldSeconds still holds the just-finished hold here.
        bumper.tapped = bumper.justReleased && bumper.heldSeconds <= tapThresholdSeconds;

        if (pressed) {
            bumper.heldSeconds += deltaTimeSeconds;
        } else {
            bumper.heldSeconds = 0.0;
        }

        boolean holding = pressed && bumper.heldSeconds > tapThresholdSeconds;
        bumper.holdStarted = holding && !bumper.wasHolding;
        bumper.holding = holding;
        bumper.wasHolding = holding;

        if (holding) {
            float progress = (float) ((bumper.heldSeconds - tapThresholdSeconds) / rampSeconds);
            bumper.analog = Math.max(0.0f, Math.min(1.0f, progress));
        } else {
            bumper.analog = 0.0f;
        }

        bumper.pressedLast = pressed;
    }
}
