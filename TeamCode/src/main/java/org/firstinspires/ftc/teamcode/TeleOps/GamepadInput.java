package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

public class GamepadInput {

    // Driver = gamepad1, Operator = gamepad2
    private final GamepadEx driverGp;
    private final GamepadEx operatorGp;

    // Driver buttons
    private final ButtonReader dLb;
    private final ButtonReader dRb;
    private final ButtonReader dB;

    // Operator buttons
    private final ButtonReader oLb;
    private final ButtonReader oRb;
    private final ButtonReader oB;

    // Per-gamepad consumption flags
    private boolean dLbSingleConsumed = false;
    private boolean dRbSingleConsumed = false;
    private boolean dComboConsumed    = false;
    private boolean dBackSingleConsumed = false;

    private boolean oLbSingleConsumed = false;
    private boolean oRbSingleConsumed = false;
    private boolean oComboConsumed    = false;
    private boolean oBackSingleConsumed    = false;

    public GamepadInput(GamepadEx driverGp, GamepadEx operatorGp) {
        this.driverGp   = driverGp;
        this.operatorGp = operatorGp;

        // Driver’s combo: LB + A
        dLb = new ButtonReader(driverGp, GamepadKeys.Button.LEFT_BUMPER);
        dRb = new ButtonReader(driverGp, GamepadKeys.Button.RIGHT_BUMPER);
        dB  = new ButtonReader(driverGp, GamepadKeys.Button.BACK);

        // Operator’s combo: LB + A (you can change to different buttons if you want)
        oLb = new ButtonReader(operatorGp, GamepadKeys.Button.LEFT_BUMPER);
        oRb = new ButtonReader(operatorGp, GamepadKeys.Button.RIGHT_BUMPER);
        oB = new ButtonReader(operatorGp, GamepadKeys.Button.BACK);
    }

    public void update() {
        // Read both pads once per loop
        driverGp.readButtons();
        operatorGp.readButtons();

        dLb.readValue();
        dRb.readValue();
        dB.readValue();

        oLb.readValue();
        oRb.readValue();
        oB.readValue();

        // Reset consumption every loop
        dLbSingleConsumed = false;
        dRbSingleConsumed = false;
        dComboConsumed    = false;
        dBackSingleConsumed = false;


        oLbSingleConsumed = false;
        oRbSingleConsumed = false;
        oComboConsumed    = false;
        oBackSingleConsumed = false;
    }

    // ==========================
    // Driver (gamepad1) methods
    // ==========================

    public boolean getDriverLbBComboPressed() {
        if (dComboConsumed) return false;

        boolean result = dLb.isDown() && dB.wasJustPressed();
        if (result) dComboConsumed = true;
        return result;
    }

    public boolean getDriverLbSinglePressed() {
        if (dLbSingleConsumed || dComboConsumed) return false;

        boolean result = dLb.wasJustPressed() && !dB.isDown();
        if (result) dLbSingleConsumed = true;
        return result;
    }

    public boolean getDriverRbSinglePressed() {
        if (dRbSingleConsumed) return false;

        boolean result = dRb.wasJustPressed() && !dB.isDown();
        if (result) dRbSingleConsumed = true;
        return result;
    }

    public boolean getDriverBackSinglePressed() {
        if (dBackSingleConsumed || dComboConsumed) return false;

        boolean result = !dLb.isDown() && dB.wasJustPressed();
        if (result) dBackSingleConsumed = true;
        return result;
    }

    // ==========================
    // Operator (gamepad2) methods
    // ==========================

    public boolean getOperatorLbBComboPressed() {
        if (oComboConsumed) return false;

        boolean result = oLb.isDown() && oB.wasJustPressed();
        if (result) oComboConsumed = true;
        return result;
    }

    public boolean getOperatorLbSinglePressed() {
        if (oLbSingleConsumed || oComboConsumed) return false;

        boolean result = oLb.wasJustPressed() && !oB.isDown();
        if (result) oLbSingleConsumed = true;
        return result;
    }

    public boolean getOperatorRbSinglePressed() {
        if (oRbSingleConsumed) return false;

        boolean result = oRb.wasJustPressed() && !oB.isDown();
        if (result) oRbSingleConsumed = true;
        return result;
    }

    public boolean getOperatorBackSinglePressed() {
        if (oBackSingleConsumed || oComboConsumed) return false;

        boolean result = !oLb.isDown() && oB.wasJustPressed();
        if (result) oBackSingleConsumed = true;
        return result;
    }
}