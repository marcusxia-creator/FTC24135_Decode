package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public enum AutoBallColors {
    GREEN,
    PURPLE,
    UNKNOWN;

    public static AutoBallColors fromHue (float hue) {
        if (greenRangeHigh[0] < hue && hue < greenRangeLow[1]) return GREEN;
        if (purpleRangeHigh[0] < hue && hue < greenRangeLow[0]) return PURPLE;
        return UNKNOWN;
    }

    public boolean isKnown() {return this != UNKNOWN;}

}