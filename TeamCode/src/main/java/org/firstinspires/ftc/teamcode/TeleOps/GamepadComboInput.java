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
    private final ButtonReader dLb, dRb, dBk,dStr;
    private final ButtonReader dA, dB, dX, dY;
    private final ButtonReader dDpl, dDpr, dDpd;

    // Operator buttons
    private final ButtonReader oLb, oRb, oBk, oStr;
    private final ButtonReader oA, oB, oX, oY;
    private final ButtonReader oDpl, oDpr, oDpd;

    // Right stick values
    private double driverRightStickX;
    private double driverRightStickY;
    private double operatorRightStickX;
    private double operatorRightStickY;

    // Left stick values
    private double driverLeftStickX;
    private double driverLeftStickY;
    private double operatorLeftStickX;
    private double operatorLeftStickY;

    //Trigger values
    private double driverLeftTrigger;
    private double driverRightTrigger;
    private double operatorLeftTrigger;
    private double operatorRightTrigger;


    // Drive input detection -Right stick boolean
    private boolean driverHasDriveInput;
    private boolean operatorHasDriveInput;
    private boolean anyHasDriveInput;

    // Combo results
    private boolean driverLbBComboPressed;
    private boolean operatorLbBComboPressed;
    private boolean operatorLbXComboPressed;

    // Single press results
    private boolean driverBackSinglePressed;
    private boolean driverLbSinglePressed;
    private boolean driverRbSinglePressed;
    private boolean driverStrSinglePressed;

    private boolean operatorBackSinglePressed;
    private boolean operatorLbSinglePressed;
    private boolean operatorRbSinglePressed;
    private boolean operatorStrSinglePressed;


    private boolean driverAPressed, driverBPressed, driverXPressed, driverYPressed;
    private boolean operatorAPressed, operatorBPressed, operatorXPressed, operatorYPressed;

    private boolean driverDpadLeftPressed, driverDpadRightPressed, driverDpadDownPressed;
    private boolean operatorDpadLeftPressed, operatorDpadRightPressed, operatorDpadDownPressed;

    // Any-pad helpers
    private boolean aPressedAny, bPressedAny, xPressedAny, yPressedAny;
    private boolean dpadLeftPressedAny, dpadRightPressedAny, dpadDownPressedAny;
    private boolean backSinglePressedAny, lbSinglePressedAny, rbSinglePressedAny;

    // Optional deadbanded values
    private double stickDeadband = 0.1;
    private double triggerThreshold = 0.7;
    private boolean bothTriggersNowAny;
    private boolean bothTriggersRisingEdgeAny;
    private boolean bothTriggersPrevAny = false;
    private boolean bothTriggersFallingEdgeAny =false;

    public GamepadComboInput(GamepadEx driverGp, GamepadEx operatorGp) {
        this.driverGp = driverGp;
        this.operatorGp = operatorGp;

        dLb = new ButtonReader(driverGp, GamepadKeys.Button.LEFT_BUMPER);
        dRb = new ButtonReader(driverGp, GamepadKeys.Button.RIGHT_BUMPER);
        dBk = new ButtonReader(driverGp, GamepadKeys.Button.BACK);
        dStr = new ButtonReader(driverGp, GamepadKeys.Button.START);

        dA = new ButtonReader(driverGp, GamepadKeys.Button.A);
        dB = new ButtonReader(driverGp, GamepadKeys.Button.B);
        dX = new ButtonReader(driverGp, GamepadKeys.Button.X);
        dY = new ButtonReader(driverGp, GamepadKeys.Button.Y);

        dDpl = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_LEFT);
        dDpr = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_RIGHT);
        dDpd = new ButtonReader(driverGp, GamepadKeys.Button.DPAD_DOWN);
        oStr = new ButtonReader(operatorGp, GamepadKeys.Button.START);

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
        // read all button status every loop
        readAll();

        // Read right stick values once per loop
        driverRightStickX = applyDeadband(driverGp.getRightX());
        driverRightStickY = applyDeadband(driverGp.getRightY());

        operatorRightStickX = applyDeadband(operatorGp.getRightX());
        operatorRightStickY = applyDeadband(operatorGp.getRightY());

        // Read Left stick values once per loop
        driverLeftStickX = applyDeadband(driverGp.getLeftX());
        driverLeftStickY = applyDeadband(driverGp.getLeftY());

        operatorLeftStickX = applyDeadband(operatorGp.getLeftX());
        operatorLeftStickY = applyDeadband(operatorGp.getLeftY());

        // Driver input detection
        driverHasDriveInput = hasDriveInput(driverGp);
        operatorHasDriveInput = hasDriveInput(operatorGp);

        anyHasDriveInput = driverHasDriveInput || operatorHasDriveInput;

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
        driverStrSinglePressed = dStr.wasJustPressed();

        operatorDpadLeftPressed = oDpl.wasJustPressed();
        operatorDpadRightPressed = oDpr.wasJustPressed();
        operatorDpadDownPressed = oDpd.wasJustPressed();
        operatorStrSinglePressed = oStr.wasJustPressed();

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
        lbSinglePressedAny = driverLbSinglePressed || operatorLbSinglePressed;
        rbSinglePressedAny = driverRbSinglePressed || operatorRbSinglePressed;

        driverLeftTrigger = driverGp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        driverRightTrigger = driverGp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        operatorLeftTrigger = operatorGp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        operatorRightTrigger = operatorGp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);


        boolean driverBoth = (driverLeftTrigger  > triggerThreshold) &&
                (driverRightTrigger > triggerThreshold);

        boolean operatorBoth = (operatorLeftTrigger  > triggerThreshold) &&
                (operatorRightTrigger > triggerThreshold);

        bothTriggersNowAny = driverBoth || operatorBoth;

        bothTriggersRisingEdgeAny  = bothTriggersNowAny && !bothTriggersPrevAny;   // press edge
        bothTriggersFallingEdgeAny = !bothTriggersNowAny && bothTriggersPrevAny;   // release edge

        bothTriggersPrevAny = bothTriggersNowAny;

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
        dStr.readValue();

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
        oStr.readValue();
    }
    public void setTriggerThreshold(double threshold) {
        triggerThreshold = threshold;
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
    public boolean getDriverStrSinglePressed() { return driverStrSinglePressed;}

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
    public boolean getOperatorStrSinglePressed() { return operatorStrSinglePressed;}

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
    public boolean getLbSinglePressedAny() {
        return lbSinglePressedAny;
    }
    public boolean getrbSinglePressedAny() {
        return rbSinglePressedAny;
    }
    public boolean getBothTriggersPressedAny() { return bothTriggersNowAny; }
    public boolean getBothTriggersRisingEdgeAny() { return bothTriggersRisingEdgeAny; }
    public boolean getBothTriggersReleasedAny() {
        return bothTriggersFallingEdgeAny;
    }

    // Driver right stick
    public double getDriverRightStickX() {return driverRightStickX;}
    public double getDriverRightStickY() {return driverRightStickY;}
    // Driver Left stick
    public double getDriverLeftStickX() {return driverLeftStickX;}
    public double getDriverLeftStickY() {return driverLeftStickY;}

    // Operator right stick
    public double getOperatorRightStickX() {return operatorRightStickX;}
    public double getOperatorRightStickY() {return operatorRightStickY;}
    // Operator Left stick
    public double getOperatorLeftStickX() {return operatorLeftStickX;}
    public double getOperatorLeftStickY() {return operatorLeftStickY;}

    //Trigger values
    public double getDriverLeftTrigger() {return driverLeftTrigger;}
    public double getDriverRightTrigger() {return driverRightTrigger;}
    public double getOperatorLeftTrigger() {return operatorLeftTrigger;}
    public double getOperatorRightTrigger() {return operatorRightTrigger;}

    //Driver Input Detection
    public boolean getDriverHasDriveInput() {
        return driverHasDriveInput;
    }
    //Operator Input Detection
    public boolean getOperatorHasDriveInput() {
        return operatorHasDriveInput;
    }

    // AnyInput Detection
    public boolean getAnyHasDriveInput() {
        return anyHasDriveInput;
    }

    //Apply deadband to stick values
    private double applyDeadband(double value) {
        if (Math.abs(value) < stickDeadband) {
            return 0.0;
        }
        return value;
    }
    // Helper to detect driver input
    private boolean hasDriveInput(GamepadEx gamepad) {
        return Math.abs(gamepad.getRightY()) > stickDeadband
                || Math.abs(gamepad.getRightX()) > stickDeadband
                || Math.abs(gamepad.getLeftX()) > stickDeadband;
    }
}