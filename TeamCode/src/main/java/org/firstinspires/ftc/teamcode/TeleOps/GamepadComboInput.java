package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

public class GamepadComboInput {

    // Driver = gamepad1, Operator = gamepad2
    private final GamepadEx driverGp;
    private final GamepadEx operatorGp;

    // Driver buttons
    private final ButtonReader dLb;
    private final ButtonReader dRb;
    private final ButtonReader dBk;
    private final ButtonReader dY;
    private final ButtonReader dX;

    // Operator buttons
    private final ButtonReader oLb;
    private final ButtonReader oRb;
    private final ButtonReader oBk;
    private final ButtonReader oX;
    private final ButtonReader oY;

    // Per-gamepad consumption flags
    private boolean dLbSingleConsumed       = false;
    private boolean dRbSingleConsumed       = false;
    private boolean dLbBkcomboConsumed      = false;
    private boolean dBackSingleConsumed     = false;
    private boolean dXSingleConsumed        = false;
    private boolean dYSingleConsumed        = false;

    private boolean oLbSingleConsumed       = false;
    private boolean oRbSingleConsumed       = false;
    private boolean oXSingleConsumed        = false;
    private boolean oYSingleConsumed        = false;
    private boolean oLbBkComboConsumed      = false;
    private boolean oLbXComboConsumed       = false;
    private boolean oBkSingleConsumed       = false;

    public GamepadComboInput(GamepadEx driverGp, GamepadEx operatorGp) {
        this.driverGp   = driverGp;
        this.operatorGp = operatorGp;

        // Driver’s combo: LB + A
        dLb = new ButtonReader(driverGp, GamepadKeys.Button.LEFT_BUMPER);
        dRb = new ButtonReader(driverGp, GamepadKeys.Button.RIGHT_BUMPER);
        dBk = new ButtonReader(driverGp, GamepadKeys.Button.BACK);
        dY  = new ButtonReader(driverGp, GamepadKeys.Button.Y);
        dX = new ButtonReader(driverGp, GamepadKeys.Button.X);

        // Operator’s combo: LB + B (you can change to different buttons if you want)

        // Operator’s combo: LB + A (you can change to different buttons if you want)
        oLb = new ButtonReader(operatorGp, GamepadKeys.Button.LEFT_BUMPER);
        oRb = new ButtonReader(operatorGp, GamepadKeys.Button.RIGHT_BUMPER);
        oBk = new ButtonReader(operatorGp, GamepadKeys.Button.BACK);
        oX = new ButtonReader(operatorGp,GamepadKeys.Button.X);
        oY = new ButtonReader(operatorGp,GamepadKeys.Button.Y);
    }

    public void update() {
        // Read both pads once per loop
        driverGp.readButtons();
        operatorGp.readButtons();

        dLb.readValue();
        dRb.readValue();
        dBk.readValue();
        dY.readValue();
        dX.readValue();


        oLb.readValue();
        oRb.readValue();
        oBk.readValue();
        oX.readValue();
        oY.readValue();

        // Reset consumption every loop
        dLbSingleConsumed   = false;
        dRbSingleConsumed   = false;
        dLbBkcomboConsumed  = false;
        dBackSingleConsumed = false;
        dXSingleConsumed    = false;
        dYSingleConsumed    = false;


        oLbSingleConsumed   = false;
        oRbSingleConsumed   = false;
        oLbBkComboConsumed  = false;
        oLbXComboConsumed   = false;
        oBkSingleConsumed   = false;
    }

    // ==========================
    // Driver (gamepad1) methods
    // ==========================

    public boolean getDriverLbBkComboPressed() {
        if (dLbBkcomboConsumed) return false;

        boolean result = dLb.isDown() && dBk.wasJustPressed();
        if (result) dLbBkcomboConsumed = true;
        return result;
    }

    public boolean getDriverLbSinglePressed() {
        if (dLbSingleConsumed || dLbBkcomboConsumed) return false;

        boolean result = dLb.wasJustPressed() && !dBk.isDown();
        if (result) dLbSingleConsumed = true;
        return result;
    }

    public boolean getDriverRbSinglePressed() {
        if (dRbSingleConsumed) return false;

        boolean result = dRb.wasJustPressed() && !dBk.isDown();
        if (result) dRbSingleConsumed = true;
        return result;
    }

    public boolean getDriverBackSinglePressed() {
        if (dBackSingleConsumed || dLbBkcomboConsumed) return false;
        boolean result = !dLb.isDown() && dBk.wasJustPressed();
        if (result) dBackSingleConsumed = true;
        return result;
    }

    public boolean getDriverXSinglePressed() {
        if (dXSingleConsumed) return false;
        boolean result = dX.isDown();
        if (result) dXSingleConsumed = true;
        return result;
    }

    public boolean getDriverYSinglePressed() {
        if (dYSingleConsumed) return false;
        boolean result = dY.isDown();
        if (result) dYSingleConsumed = true;
        return result;
    }

    // ==========================
    // Operator (gamepad2) methods
    // ==========================

    public boolean getOperatorLbBComboPressed() {
        if (oLbBkComboConsumed) return false;

        boolean result = oLb.isDown() && oBk.wasJustPressed();
        if (result) oLbBkComboConsumed = true;
        return result;
    }

    public boolean getOperatorLbXComboPressed() {
        if (oLbXComboConsumed) return false;

        boolean result = oLb.isDown() && oX.wasJustPressed();
        if (result) oLbXComboConsumed = true;
        return result;
    }

    public boolean getOperatorLbSinglePressed() {
        if (oLbSingleConsumed || oLbBkComboConsumed) return false;

        boolean result = oLb.wasJustPressed() && !oBk.isDown();
        if (result) oLbSingleConsumed = true;
        return result;
    }

    public boolean getOperatorRbSinglePressed() {
        if (oRbSingleConsumed) return false;

        boolean result = oRb.wasJustPressed() && !oBk.isDown();
        if (result) oRbSingleConsumed = true;
        return result;
    }

    public boolean getOperatorBackSinglePressed() {
        if (oBkSingleConsumed || oLbBkComboConsumed ) return false;

        boolean result = !oLb.isDown() && oBk.wasJustPressed();
        if (result) oBkSingleConsumed = true;
        return result;
    }

    public boolean getOperatorXSinglePressed() {
        if (oXSingleConsumed) return false;
        boolean result = oX.isDown();
        if (result) oXSingleConsumed = true;
        return result;
    }

    public boolean getOperatorYSinglePressed() {
        if (oYSingleConsumed) return false;
        boolean result = oY.isDown();
        if (result) oYSingleConsumed = true;
        return result;
    }
}