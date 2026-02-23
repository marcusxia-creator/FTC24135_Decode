package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

/**
 * Compute-once-per-loop input resolver for driver + operator gamepads.
 *
 * Requirements:
 * - Call gamepad.readButtons() ONCE per loop in TeleOp.
 * - Then call update() exactly once per loop.
 * - This class does NOT call readButtons().
 */
public class GamepadComboInput {

    private final GamepadEx driverGp;
    private final GamepadEx operatorGp;

    // Driver buttons
    private final ButtonReader dLb, dRb, dBk;
    private final ButtonReader dA, dB, dX, dY;
    private final ButtonReader dDpl, dDpr, dDpd;

    // Operator buttons
    private final ButtonReader oLb, oRb, oBk;
    private final ButtonReader oA, oB, oX, oY;
    private final ButtonReader oDpl, oDpr, oDpd;

    // Combo results
    private boolean driverLbBComboPressed;
    private boolean operatorLbBComboPressed;
    private boolean operatorLbXComboPressed;

    // Single press results
    private boolean driverBackSinglePressed;
    private boolean driverLbSinglePressed;
    private boolean driverRbSinglePressed;

    private boolean operatorBackSinglePressed;
    private boolean operatorLbSinglePressed;
    private boolean operatorRbSinglePressed;

    private boolean driverAPressed, driverBPressed, driverXPressed, driverYPressed;
    private boolean operatorAPressed, operatorBPressed, operatorXPressed, operatorYPressed;

    private boolean driverDpadLeftPressed, driverDpadRightPressed, driverDpadDownPressed;
    private boolean operatorDpadLeftPressed, operatorDpadRightPressed, operatorDpadDownPressed;

    // Any-pad helpers
    private boolean aPressedAny, bPressedAny, xPressedAny, yPressedAny;
    private boolean dpadLeftPressedAny, dpadRightPressedAny, dpadDownPressedAny;
    private boolean backSinglePressedAny;

    public GamepadComboInput(GamepadEx driverGp, GamepadEx operatorGp) {
        this.driverGp = driverGp;
        this.operatorGp = operatorGp;

        dLb = new ButtonReader(driverGp, GamepadKeys.Button.LEFT_BUMPER);
        dRb = new ButtonReader(driverGp, GamepadKeys.Button.RIGHT_BUMPER);
        dBk = new ButtonReader(driverGp, GamepadKeys.Button.BACK);

        dA = new ButtonReader(driverGp, GamepadKeys.Button.A);
        dB = new ButtonReader(driverGp, GamepadKeys.Button.B);
        dX = new ButtonReader(driverGp, GamepadKeys.Button.X);
        dY = new ButtonReader(driverGp, GamepadKeys.Button.Y);

        dDpl = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_LEFT);
        dDpr = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_RIGHT);
        dDpd = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_DOWN);

        oLb = new ButtonReader(operatorGp, GamepadKeys.Button.LEFT_BUMPER);
        oRb = new ButtonReader(operatorGp, GamepadKeys.Button.RIGHT_BUMPER);
        oBk = new ButtonReader(operatorGp, GamepadKeys.Button.BACK);

        oA = new ButtonReader(operatorGp, GamepadKeys.Button.A);
        oB = new ButtonReader(operatorGp, GamepadKeys.Button.B);
        oX = new ButtonReader(operatorGp, GamepadKeys.Button.X);
        oY = new ButtonReader(operatorGp, GamepadKeys.Button.Y);

        oDpl = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_LEFT);
        oDpr = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_RIGHT);
        oDpd = new ButtonReader(operatorGp, GamepadKeys.Button.DPAD_DOWN);
    }

    public void update() {

        readAll();

        // Driver combos and singles
        driverLbBComboPressed = dLb.isDown() && dBk.wasJustPressed();

        driverBackSinglePressed = !dLb.isDown() && dBk.wasJustPressed();
        driverLbSinglePressed = dLb.wasJustPressed() && !dBk.isDown();
        driverRbSinglePressed = dRb.wasJustPressed() && !dBk.isDown();

        if (driverLbBComboPressed) {
            driverBackSinglePressed = false;
            driverLbSinglePressed = false;
        }

        // Operator combos and singles
        operatorLbBComboPressed = oLb.isDown() && oBk.wasJustPressed();
        operatorLbXComboPressed = oLb.isDown() && oX.wasJustPressed();

        operatorBackSinglePressed = !oLb.isDown() && oBk.wasJustPressed();
        operatorLbSinglePressed = oLb.wasJustPressed() && !oBk.isDown() && !oX.isDown();
        operatorRbSinglePressed = oRb.wasJustPressed() && !oBk.isDown();

        if (operatorLbBComboPressed || operatorLbXComboPressed) {
            operatorBackSinglePressed = false;
            operatorLbSinglePressed = false;
        }

        // Single presses
        driverAPressed = dA.wasJustPressed();
        driverBPressed = dB.wasJustPressed();
        driverXPressed = dX.wasJustPressed();
        driverYPressed = dY.wasJustPressed();

        operatorAPressed = oA.wasJustPressed();
        operatorBPressed = oB.wasJustPressed();
        operatorXPressed = oX.wasJustPressed();
        operatorYPressed = oY.wasJustPressed();

        driverDpadLeftPressed = dDpl.wasJustPressed();
        driverDpadRightPressed = dDpr.wasJustPressed();
        driverDpadDownPressed = dDpd.wasJustPressed();

        operatorDpadLeftPressed = oDpl.wasJustPressed();
        operatorDpadRightPressed = oDpr.wasJustPressed();
        operatorDpadDownPressed = oDpd.wasJustPressed();

        if (operatorLbXComboPressed) {
            operatorXPressed = false;
        }

        // Any-pad aggregation
        aPressedAny = driverAPressed || operatorAPressed;
        bPressedAny = driverBPressed || operatorBPressed;
        xPressedAny = driverXPressed || operatorXPressed;
        yPressedAny = driverYPressed || operatorYPressed;

        dpadLeftPressedAny = driverDpadLeftPressed || operatorDpadLeftPressed;
        dpadRightPressedAny = driverDpadRightPressed || operatorDpadRightPressed;
        dpadDownPressedAny = driverDpadDownPressed || operatorDpadDownPressed;

        backSinglePressedAny = driverBackSinglePressed || operatorBackSinglePressed;
    }

    private void readAll() {
        dLb.readValue();
        dRb.readValue();
        dBk.readValue();
        dA.readValue();
        dB.readValue();
        dX.readValue();
        dY.readValue();
        dDpl.readValue();
        dDpr.readValue();
        dDpd.readValue();

        oLb.readValue();
        oRb.readValue();
        oBk.readValue();
        oA.readValue();
        oB.readValue();
        oX.readValue();
        oY.readValue();
        oDpl.readValue();
        oDpr.readValue();
        oDpd.readValue();
    }

    public boolean getDriverLbBComboPressed() {
        return driverLbBComboPressed;
    }

    public boolean getDriverLbSinglePressed() {
        return driverLbSinglePressed;
    }

    public boolean getDriverRbSinglePressed() {
        return driverRbSinglePressed;
    }

    public boolean getDriverBackSinglePressed() {
        return driverBackSinglePressed;
    }

    public boolean getOperatorLbBComboPressed() {
        return operatorLbBComboPressed;
    }

    public boolean getOperatorLbXComboPressed() {
        return operatorLbXComboPressed;
    }

    public boolean getOperatorLbSinglePressed() {
        return operatorLbSinglePressed;
    }

    public boolean getOperatorRbSinglePressed() {
        return operatorRbSinglePressed;
    }

    public boolean getOperatorBackSinglePressed() {
        return operatorBackSinglePressed;
    }

    public boolean getAPressedAny() {
        return aPressedAny;
    }

    public boolean getBPressedAny() {
        return bPressedAny;
    }

    public boolean getXPressedAny() {
        return xPressedAny;
    }

    public boolean getYPressedAny() {
        return yPressedAny;
    }

    public boolean getDpadLeftPressedAny() {
        return dpadLeftPressedAny;
    }

    public boolean getDpadRightPressedAny() {
        return dpadRightPressedAny;
    }

    public boolean getDpadDownPressedAny() {
        return dpadDownPressedAny;
    }

    public boolean getBackSinglePressedAny() {
        return backSinglePressedAny;
    }
}