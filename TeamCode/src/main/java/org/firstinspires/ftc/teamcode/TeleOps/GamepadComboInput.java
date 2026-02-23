package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

/**
 * Compute-once-per-loop input resolver for two gamepads (driver + operator).
 *
 * - TeleOp must call readButtons() ONCE per loop for both gamepads.
 * - This class does NOT call readButtons().
 * - update() computes "just pressed" pulses for singles + combos.
 * - Combos override overlapping singles in the same loop.
 */
public class GamepadComboInput {

    private final GamepadEx driverGp;
    private final GamepadEx operatorGp;

    // -------------------------
    // Driver ButtonReaders
    // -------------------------
    private final ButtonReader dLb, dRb, dBk;
    private final ButtonReader dX, dY;
    private final ButtonReader dDpl, dDpr, dDpd;

    // -------------------------
    // Operator ButtonReaders
    // -------------------------
    private final ButtonReader oLb, oRb, oBk;
    private final ButtonReader oX, oY;
    private final ButtonReader oDpl, oDpr, oDpd;

    // ==========================
    // Computed results (one-loop pulses)
    // ==========================

    // Existing API results (kept)
    private boolean driverLbBComboPressed;
    private boolean driverLbSinglePressed;
    private boolean driverRbSinglePressed;
    private boolean driverBackSinglePressed;

    private boolean operatorLbBComboPressed;
    private boolean operatorLbXComboPressed;
    private boolean operatorLbSinglePressed;
    private boolean operatorRbSinglePressed;
    private boolean operatorBackSinglePressed;

    // New single presses (per-pad)
    private boolean driverXPressed, driverYPressed, driverDpadLeftPressed, driverDpadRightPressed, driverDpadDownPressed;
    private boolean operatorXPressed, operatorYPressed, operatorDpadLeftPressed, operatorDpadRightPressed, operatorDpadDownPressed;

    // Optional “ANY” helpers (either pad)
    private boolean xPressedAny, yPressedAny, dpadLeftPressedAny, dpadRightPressedAny, dpadDownPressedAny, backPressedAny;

    public GamepadComboInput(GamepadEx driverGp, GamepadEx operatorGp) {
        this.driverGp   = driverGp;
        this.operatorGp = operatorGp;

        // Driver
        dLb  = new ButtonReader(driverGp, GamepadKeys.Button.LEFT_BUMPER);
        dRb  = new ButtonReader(driverGp, GamepadKeys.Button.RIGHT_BUMPER);
        dBk  = new ButtonReader(driverGp, GamepadKeys.Button.BACK);
        dX   = new ButtonReader(driverGp, GamepadKeys.Button.X);
        dY   = new ButtonReader(driverGp, GamepadKeys.Button.Y);
        dDpl = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_LEFT);
        dDpr = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_RIGHT);
        dDpd = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_DOWN);

        // Operator
        oLb  = new ButtonReader(operatorGp, GamepadKeys.Button.LEFT_BUMPER);
        oRb  = new ButtonReader(operatorGp, GamepadKeys.Button.RIGHT_BUMPER);
        oBk  = new ButtonReader(operatorGp, GamepadKeys.Button.BACK);
        oX   = new ButtonReader(operatorGp, GamepadKeys.Button.X);
        oY   = new ButtonReader(operatorGp, GamepadKeys.Button.Y);
        oDpl = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_LEFT);
        oDpr = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_RIGHT);
        oDpd = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_DOWN);
    }

    /** Call once per loop AFTER both pads have readButtons(). */
    public void update() {
        // Update all readers
        readAll();

        // ---- DRIVER combos/singles ----
        driverLbBComboPressed = dLb.isDown() && dBk.wasJustPressed();

        driverBackSinglePressed = (!dLb.isDown()) && dBk.wasJustPressed();
        driverLbSinglePressed   = dLb.wasJustPressed() && !dBk.isDown();
        driverRbSinglePressed   = dRb.wasJustPressed() && !dBk.isDown();

        if (driverLbBComboPressed) {
            driverBackSinglePressed = false;
            driverLbSinglePressed   = false;
        }

        // ---- OPERATOR combos/singles ----
        operatorLbBComboPressed = oLb.isDown() && oBk.wasJustPressed();
        operatorLbXComboPressed = oLb.isDown() && oX.wasJustPressed();

        operatorBackSinglePressed = (!oLb.isDown()) && oBk.wasJustPressed();
        operatorLbSinglePressed   = oLb.wasJustPressed() && !oBk.isDown() && !oX.isDown();
        operatorRbSinglePressed   = oRb.wasJustPressed() && !oBk.isDown();

        if (operatorLbBComboPressed || operatorLbXComboPressed) {
            operatorBackSinglePressed = false;
            operatorLbSinglePressed   = false;
        }

        // ---- New: single press buttons ----
        // If you want to “reserve” X when LB+X combo fires, suppress operatorXPressed.
        driverXPressed = dX.wasJustPressed();
        driverYPressed = dY.wasJustPressed();
        driverDpadLeftPressed  = dDpl.wasJustPressed();
        driverDpadRightPressed = dDpr.wasJustPressed();
        driverDpadDownPressed  = dDpd.wasJustPressed();

        operatorXPressed = oX.wasJustPressed();
        operatorYPressed = oY.wasJustPressed();
        operatorDpadLeftPressed  = oDpl.wasJustPressed();
        operatorDpadRightPressed = oDpr.wasJustPressed();
        operatorDpadDownPressed  = oDpd.wasJustPressed();

        // Combo priority example: if LB+X fired, then X shouldn't also count as a single press
        if (operatorLbXComboPressed) {
            operatorXPressed = false;
        }

        // ---- Any-pad helpers ----
        xPressedAny = driverXPressed || operatorXPressed;
        yPressedAny = driverYPressed || operatorYPressed;
        dpadLeftPressedAny  = driverDpadLeftPressed  || operatorDpadLeftPressed;
        dpadRightPressedAny = driverDpadRightPressed || operatorDpadRightPressed;
        dpadDownPressedAny  = driverDpadDownPressed  || operatorDpadDownPressed;
        backPressedAny = driverBackSinglePressed || operatorBackSinglePressed; // single-back only
    }

    private void readAll() {
        dLb.readValue(); dRb.readValue(); dBk.readValue();
        dX.readValue();  dY.readValue();
        dDpl.readValue(); dDpr.readValue(); dDpd.readValue();

        oLb.readValue(); oRb.readValue(); oBk.readValue();
        oX.readValue();  oY.readValue();
        oDpl.readValue(); oDpr.readValue(); oDpd.readValue();
    }

    // ==========================
    // Existing public API — unchanged
    // ==========================
    public boolean getDriverLbBComboPressed() { return driverLbBComboPressed; }
    public boolean getDriverLbSinglePressed() { return driverLbSinglePressed; }
    public boolean getDriverRbSinglePressed() { return driverRbSinglePressed; }
    public boolean getDriverBackSinglePressed() { return driverBackSinglePressed; }

    public boolean getOperatorLbBComboPressed() { return operatorLbBComboPressed; }
    public boolean getOperatorLbXComboPressed() { return operatorLbXComboPressed; }
    public boolean getOperatorLbSinglePressed() { return operatorLbSinglePressed; }
    public boolean getOperatorRbSinglePressed() { return operatorRbSinglePressed; }
    public boolean getOperatorBackSinglePressed() { return operatorBackSinglePressed; }

    // ==========================
    // New getters (per-pad)
    // ==========================
    public boolean getDriverXPressed() { return driverXPressed; }
    public boolean getDriverYPressed() { return driverYPressed; }
    public boolean getDriverDpadLeftPressed() { return driverDpadLeftPressed; }
    public boolean getDriverDpadRightPressed() { return driverDpadRightPressed; }
    public boolean getDriverDpadDownPressed() { return driverDpadDownPressed; }

    public boolean getOperatorXPressed() { return operatorXPressed; }
    public boolean getOperatorYPressed() { return operatorYPressed; }
    public boolean getOperatorDpadLeftPressed() { return operatorDpadLeftPressed; }
    public boolean getOperatorDpadRightPressed() { return operatorDpadRightPressed; }
    public boolean getOperatorDpadDownPressed() { return operatorDpadDownPressed; }

    // ==========================
    // New helpers (either pad)
    // ==========================
    public boolean getXPressedAny() { return xPressedAny; }
    public boolean getYPressedAny() { return yPressedAny; }
    public boolean getDpadLeftPressedAny() { return dpadLeftPressedAny; }
    public boolean getDpadRightPressedAny() { return dpadRightPressedAny; }
    public boolean getDpadDownPressedAny() { return dpadDownPressedAny; }
    public boolean getBackSinglePressedAny() { return backPressedAny; }
}