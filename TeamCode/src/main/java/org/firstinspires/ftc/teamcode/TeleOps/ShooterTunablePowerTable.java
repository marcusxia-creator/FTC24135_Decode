package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

import java.util.TreeMap;
@Config
public class ShooterTunablePowerTable {
    // These static variables are tunable via FTC Dashboard
    public static double DIST1 = 60.0;
    public static double POWER1 = 0.70;
    public static double DIST2 = 80.0;
    public static double POWER2 = 0.75;
    public static double DIST3 = 110.0;
    public static double POWER3 = 0.86;

    private final TreeMap<Double, Double> table = new TreeMap<>();

    /**
     * Constructor for the power table.
     * The table will be dynamically updated from the Dashboard values.
     */
    public ShooterTunablePowerTable() {
        // The constructor can be empty, as the update() method will handle populating the table.
    }

    /**
     * FIX: Rebuilds the TreeMap with the latest static values from FTC Dashboard.
     * This ensures that any live tuning is immediately reflected in the calculations.
     */
    public void update() {
        table.clear();
        table.put(DIST1, POWER1);
        table.put(DIST2, POWER2);
        table.put(DIST3, POWER3);
    }

    /**
     * Calculates the appropriate shooter power for a given distance using linear interpolation.
     * It ensures the data is updated from the FTC Dashboard before performing the calculation.
     *
     * @param distance The current distance to the target in the same units as DIST1, DIST2, etc.
     * @return The interpolated motor power (from 0.0 to 1.0).
     */
    public double getPower(double distance) {
        // FIX: Always call update() first to get the latest dashboard values.
        update();

        // Find the known distance points just below and just above the current distance.
        Double lower = table.floorKey(distance);
        Double upper = table.ceilingKey(distance);

        // Handle edge cases where the distance is outside the defined range.
        if (lower == null && upper == null) return 0.0; // Return 0 if the table is empty.
        if (lower == null) return table.get(upper); // If below range, use the lowest power.
        if (upper == null) return table.get(lower); // If above range, use the highest power.

        // If the distance is an exact match, no need to interpolate.
        if (lower.equals(upper)) return table.get(distance);

        // Perform linear interpolation.
        double lowerVal = table.get(lower);
        double upperVal = table.get(upper);

        // This calculates the proportional power based on where the distance falls between the two points.
        return lowerVal + (distance - lower) * (upperVal - lowerVal) / (upper - lower);
    }
}
