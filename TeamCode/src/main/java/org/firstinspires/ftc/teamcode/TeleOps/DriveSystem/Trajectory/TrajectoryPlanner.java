package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.TrapezoidalProfile;

import java.util.List;

/**
 * Decides how fast the robot should travel at each point of a
 * multi-segment trajectory, then builds each segment's MotionProfile.
 *
 * This is a one-shot calculation over an already-built list of
 * TrajectorySegments, run once when a trajectory is assembled -- not a
 * per-loop object -- so it is written as a static utility class rather
 * than something you construct and keep around.
 *
 * Algorithm:
 *
 * 1. Cornering cap: sample curvature along each segment's DrivePath
 *    and cap that segment's cruise speed using a centripetal
 *    acceleration limit: v <= sqrt(maxLateralAccel / curvature).
 *    Straight segments (curvature = 0) get no cap.
 *
 * 2. Junction velocities: the velocity at each waypoint between two
 *    segments is limited by whichever neighboring segment's cornering
 *    cap is tighter.
 *
 * 3. Forward pass (acceleration-limited) then backward pass
 *    (deceleration-limited) over the junction velocities, using
 *    v^2 = v0^2 + 2*a*d, so the robot never gets a velocity target it
 *    physically cannot reach or stop from in time. This is the same
 *    velocity-constraint-chaining idea used by other trajectory
 *    planners (RoadRunner, Pathfinder, etc).
 *
 * 4. Each segment's TrapezoidalProfile is built using its boundary
 *    velocities and its cornering cap as the cruise velocity.
 *
 * Limitation: TrapezoidalProfile can only represent one
 * accelerate/cruise/decelerate triplet, so step 1 applies a single,
 * conservative cornering cap across the whole segment (the tightest
 * curvature found anywhere in it), not a continuously varying speed.
 * If a CubicBezierPath's curvature varies a lot, split it into several
 * shorter segments with TrajectoryBuilder -- CompositeDrivePath
 * already supports that for free, and each shorter segment gets its
 * own tighter cap.
 *
 * Units:
 * - distance: millimetres
 * - velocity: millimetres per second
 * - acceleration: millimetres per second squared
 */
@Config
public final class TrajectoryPlanner {

    /**
     * Maximum sideways (centripetal) acceleration allowed while
     * cornering. Needs tuning on a real robot.
     */
    public static double maximumLateralAccelerationMMPerSecondSquared = 900.0;

    /**
     * Number of curvature samples taken along each segment when
     * looking for its worst-case (tightest-radius) corner.
     */
    public static int curvatureSampleCount = 20;

    private TrajectoryPlanner() {
    }

    /**
     * Plans a trajectory that comes to a full stop at the end.
     */
    public static void plan(List<TrajectorySegment> segments) {
        plan(segments, 0.0);
    }

    /**
     * Assigns start/end velocities, start distance/time offsets, and
     * a MotionProfile to every segment in the list, in place.
     *
     * @param segments                    ordered trajectory segments
     * @param finalEndVelocityMMPerSecond velocity the robot should
     *                                     have when it reaches the
     *                                     very last segment's end
     */
    public static void plan(
            List<TrajectorySegment> segments,
            double finalEndVelocityMMPerSecond
    ) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException(
                    "TrajectoryPlanner requires at least one segment."
            );
        }

        int segmentCount = segments.size();

        double[] corneringCapMMPerSecond = new double[segmentCount];

        for (int index = 0; index < segmentCount; index++) {
            TrajectorySegment segment = segments.get(index);

            corneringCapMMPerSecond[index] = Math.min(
                    segment.getMaximumVelocityMMPerSecond(),
                    calculateCorneringCapMMPerSecond(
                            segment.getDrivePath()
                    )
            );
        }

        /*
         * junctionVelocity[i] is the velocity at the boundary between
         * segment i-1 and segment i. There are segmentCount + 1
         * junctions: before the first segment and after every segment.
         */
        double[] junctionVelocity = new double[segmentCount + 1];

        junctionVelocity[0] = 0.0;
        junctionVelocity[segmentCount] =
                Math.max(0.0, finalEndVelocityMMPerSecond);

        for (int junction = 1; junction < segmentCount; junction++) {
            junctionVelocity[junction] = Math.min(
                    corneringCapMMPerSecond[junction - 1],
                    corneringCapMMPerSecond[junction]
            );
        }

        /*
         * Forward pass: an acceleration-limited ramp-up cannot exceed
         * what the segment's own cornering cap already allows.
         */
        for (int index = 0; index < segmentCount; index++) {
            TrajectorySegment segment = segments.get(index);

            double reachableVelocity = Math.sqrt(
                    junctionVelocity[index] * junctionVelocity[index]
                            + 2.0
                            * segment.getMaximumAccelerationMMPerSecondSquared()
                            * segment.getLengthMM()
            );

            junctionVelocity[index + 1] = Math.min(
                    junctionVelocity[index + 1],
                    reachableVelocity
            );
        }

        /*
         * Backward pass: a deceleration-limited ramp-down guarantees
         * the robot can always slow down in time for the next corner
         * or the final stop.
         */
        for (int index = segmentCount - 1; index >= 0; index--) {
            TrajectorySegment segment = segments.get(index);

            double reachableVelocity = Math.sqrt(
                    junctionVelocity[index + 1] * junctionVelocity[index + 1]
                            + 2.0
                            * segment.getMaximumDecelerationMMPerSecondSquared()
                            * segment.getLengthMM()
            );

            junctionVelocity[index] = Math.min(
                    junctionVelocity[index],
                    reachableVelocity
            );
        }

        double distanceSoFarMM = 0.0;
        double timeSoFarSeconds = 0.0;

        for (int index = 0; index < segmentCount; index++) {
            TrajectorySegment segment = segments.get(index);

            segment.setStartVelocityMMPerSecond(junctionVelocity[index]);
            segment.setEndVelocityMMPerSecond(junctionVelocity[index + 1]);

            segment.setStartDistanceAlongTrajectoryMM(distanceSoFarMM);
            segment.setStartTimeSeconds(timeSoFarSeconds);

            TrapezoidalProfile profile = new TrapezoidalProfile(
                    segment.getLengthMM(),
                    junctionVelocity[index],
                    junctionVelocity[index + 1],
                    corneringCapMMPerSecond[index],
                    segment.getMaximumAccelerationMMPerSecondSquared(),
                    segment.getMaximumDecelerationMMPerSecondSquared()
            );

            segment.setMotionProfile(profile);

            /*
             * getEndDistanceAlongTrajectoryMM()/getEndTimeSeconds()
             * both need the motion profile that was just set, so this
             * must happen after setMotionProfile(), not before.
             */
            distanceSoFarMM = segment.getEndDistanceAlongTrajectoryMM();
            timeSoFarSeconds = segment.getEndTimeSeconds();
        }
    }

    /**
     * Finds the tightest (largest absolute) curvature anywhere along a
     * path and converts it into a cornering speed cap using a
     * centripetal-acceleration limit.
     */
    private static double calculateCorneringCapMMPerSecond(
            DrivePath path
    ) {
        double lengthMM = path.getLengthMM();

        int sampleCount = Math.max(2, curvatureSampleCount);

        double maximumAbsoluteCurvaturePerMM = 0.0;

        for (int sample = 0; sample < sampleCount; sample++) {
            double distanceMM =
                    lengthMM * sample / (double) (sampleCount - 1);

            double curvaturePerMM = Math.abs(
                    path.getPathState(distanceMM).getCurvaturePerMM()
            );

            maximumAbsoluteCurvaturePerMM = Math.max(
                    maximumAbsoluteCurvaturePerMM,
                    curvaturePerMM
            );
        }

        if (maximumAbsoluteCurvaturePerMM < 1e-9) {
            /*
             * Straight path: no cornering limit.
             */
            return Double.POSITIVE_INFINITY;
        }

        return Math.sqrt(
                maximumLateralAccelerationMMPerSecondSquared
                        / maximumAbsoluteCurvaturePerMM
        );
    }
}
