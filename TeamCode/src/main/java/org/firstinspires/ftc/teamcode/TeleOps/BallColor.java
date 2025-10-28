package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.HashMap;
import java.util.Map;


public enum BallColor {
    PURPLE,
    GREEN,
    UNKNOWN;

    // Optional: use a static lookup for string → enum conversion
    private static final Map<String, BallColor> NAME_MAP = new HashMap<>();
    static {
        for (BallColor c : values()) NAME_MAP.put(c.name(), c);
    }

    public static BallColor fromString(String colorName) {
        if (colorName == null) return UNKNOWN;
        BallColor color = NAME_MAP.get(colorName.toUpperCase());
        return (color != null) ? color : UNKNOWN;
    }
}
