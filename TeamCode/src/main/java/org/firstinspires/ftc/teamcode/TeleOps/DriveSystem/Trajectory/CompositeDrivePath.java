package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePathState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Combines multiple DrivePath segments into one continuous DrivePath.
 *
 * Example:
 *
 *      Segment 0: A -> B
 *      Segment 1: B -> C
 *      Segment 2: C -> D
 *
 * The CompositeDrivePath presents all three segments as:
 *
 *      A -> B -> C -> D
 *
 * The caller uses one global distance measured from the beginning
 * of the complete path.
 *
 * Units:
 * - position: mm
 * - distance: mm
 */
public final class CompositeDrivePath implements DrivePath {

    /**
     * Allowed position difference between the end of one segment
     * and the beginning of the next segment.
     */
    private static final double CONNECTION_TOLERANCE_MM = 1.0;

    /**
     * Ordered path segments.
     */
    private final List<DrivePath> pathSegments;

    /**
     * Starting global distance of each segment.
     *
     * Example:
     *
     * Segment 0 length = 500 mm
     * Segment 1 length = 300 mm
     * Segment 2 length = 700 mm
     *
     * segmentStartDistanceMM:
     *
     * [0] = 0
     * [1] = 500
     * [2] = 800
     */
    private final double[] segmentStartDistanceMM;

    /**
     * Total physical length of all path segments.
     */
    private final double totalLengthMM;

    /**
     * Creates a composite path from a list of DrivePath segments.
     */
    public CompositeDrivePath(
            List<DrivePath> pathSegments
    ) {

        if (pathSegments == null) {
            throw new IllegalArgumentException(
                    "Path segment list cannot be null."
            );
        }

        if (pathSegments.isEmpty()) {
            throw new IllegalArgumentException(
                    "CompositeDrivePath requires at least one path segment."
            );
        }

        /*
         * Copy the list so external code cannot modify the
         * CompositeDrivePath after construction.
         */
        this.pathSegments =
                new ArrayList<>(pathSegments);

        /*
         * Make sure no segment is null.
         */
        for (DrivePath path : this.pathSegments) {

            if (path == null) {
                throw new IllegalArgumentException(
                        "Path segments cannot contain null."
                );
            }
        }

        /*
         * Verify that each segment connects to the next segment.
         */
        validateConnections();

        /*
         * Calculate the global starting distance of every segment.
         */
        segmentStartDistanceMM =
                new double[this.pathSegments.size()];

        double accumulatedDistanceMM = 0.0;

        for (int index = 0;
             index < this.pathSegments.size();
             index++) {

            segmentStartDistanceMM[index] =
                    accumulatedDistanceMM;

            accumulatedDistanceMM +=
                    this.pathSegments
                            .get(index)
                            .getLengthMM();
        }

        totalLengthMM =
                accumulatedDistanceMM;
    }

    /**
     * Convenience constructor.
     *
     * Allows:
     *
     * new CompositeDrivePath(
     *      path1,
     *      path2,
     *      path3
     * );
     */
    public CompositeDrivePath(
            DrivePath... pathSegments
    ) {

        if (pathSegments == null
                || pathSegments.length == 0) {

            throw new IllegalArgumentException(
                    "CompositeDrivePath requires at least one path segment."
            );
        }

        List<DrivePath> pathList =
                new ArrayList<>();

        Collections.addAll(
                pathList,
                pathSegments
        );

        /*
         * Since Java requires this(...) to be the first statement,
         * this constructor cannot directly call the main constructor
         * after creating pathList.
         *
         * Therefore, use the private helper constructor below.
         */
        CompositeDrivePath temporary =
                new CompositeDrivePath(pathList);

        this.pathSegments =
                temporary.pathSegments;

        this.segmentStartDistanceMM =
                temporary.segmentStartDistanceMM;

        this.totalLengthMM =
                temporary.totalLengthMM;
    }

    /**
     * Returns the DrivePathState at a global distance measured from
     * the beginning of the entire composite path.
     *
     * Example:
     *
     * Segment 0 length = 500 mm
     * Segment 1 length = 300 mm
     *
     * Request:
     *
     *      getPathState(650)
     *
     * means:
     *
     *      Segment 1
     *      local distance = 650 - 500 = 150 mm
     */
    @Override
    public DrivePathState getPathState(
            double distanceAlongPathMM
    ) {

        /*
         * Handle path beginning.
         */
        if (distanceAlongPathMM <= 0.0) {

            DrivePath firstPath =
                    pathSegments.get(0);

            DrivePathState localState =
                    firstPath.getPathState(0.0);

            return createGlobalState(
                    0.0,
                    localState
            );
        }

        /*
         * Handle path end.
         */
        if (distanceAlongPathMM >= totalLengthMM) {

            int lastIndex =
                    pathSegments.size() - 1;

            DrivePath lastPath =
                    pathSegments.get(lastIndex);

            DrivePathState localState =
                    lastPath.getPathState(
                            lastPath.getLengthMM()
                    );

            return createGlobalState(
                    totalLengthMM,
                    localState
            );
        }

        /*
         * Determine which segment contains the requested global
         * distance.
         */
        int segmentIndex =
                findSegmentIndex(
                        distanceAlongPathMM
                );

        DrivePath activeSegment =
                pathSegments.get(
                        segmentIndex
                );

        double segmentStartDistance =
                segmentStartDistanceMM[
                        segmentIndex
                        ];

        /*
         * Convert global trajectory distance to local segment distance.
         */
        double localDistanceMM =
                distanceAlongPathMM
                        - segmentStartDistance;

        DrivePathState localState =
                activeSegment.getPathState(
                        localDistanceMM
                );

        /*
         * Convert the local DrivePathState into a global one.
         */
        return createGlobalState(
                distanceAlongPathMM,
                localState
        );
    }

    /**
     * Finds which path segment contains the requested global distance.
     */
    private int findSegmentIndex(
            double distanceAlongPathMM
    ) {

        /*
         * Start from the last segment and search backward.
         *
         * For the small number of FTC trajectory segments,
         * this is inexpensive and easy to understand.
         */
        for (int index =
             pathSegments.size() - 1;
             index >= 0;
             index--) {

            if (distanceAlongPathMM
                    >= segmentStartDistanceMM[index]) {

                return index;
            }
        }

        return 0;
    }

    /**
     * Converts a segment-local DrivePathState into a global
     * CompositeDrivePath state.
     */
    private DrivePathState createGlobalState(
            double globalDistanceMM,
            DrivePathState localState
    ) {

        double globalProgress;

        if (totalLengthMM > 0.0) {

            globalProgress =
                    globalDistanceMM
                            / totalLengthMM;

        } else {

            globalProgress = 1.0;
        }

        return new DrivePathState(
                globalDistanceMM,
                globalProgress,

                localState.getXMM(),
                localState.getYMM(),

                localState.getTangentX(),
                localState.getTangentY(),
                localState.getTangentHeadingRadians(),

                localState.getCurvaturePerMM()
        );
    }

    /**
     * Verifies that the end of every path segment matches the
     * beginning of the next segment.
     */
    private void validateConnections() {

        for (int index = 0;
             index < pathSegments.size() - 1;
             index++) {

            DrivePath currentPath =
                    pathSegments.get(index);

            DrivePath nextPath =
                    pathSegments.get(index + 1);

            double xDifferenceMM =
                    nextPath.getStartXMM()
                            - currentPath.getEndXMM();

            double yDifferenceMM =
                    nextPath.getStartYMM()
                            - currentPath.getEndYMM();

            double connectionErrorMM =
                    Math.sqrt(
                            xDifferenceMM * xDifferenceMM
                                    + yDifferenceMM * yDifferenceMM
                    );

            if (connectionErrorMM
                    > CONNECTION_TOLERANCE_MM) {

                throw new IllegalArgumentException(
                        "Path segment "
                                + index
                                + " does not connect to segment "
                                + (index + 1)
                                + ". Connection error = "
                                + connectionErrorMM
                                + " mm."
                );
            }
        }
    }

    @Override
    public double getLengthMM() {
        return totalLengthMM;
    }

    @Override
    public double getStartXMM() {
        return pathSegments
                .get(0)
                .getStartXMM();
    }

    @Override
    public double getStartYMM() {
        return pathSegments
                .get(0)
                .getStartYMM();
    }

    @Override
    public double getEndXMM() {

        int lastIndex =
                pathSegments.size() - 1;

        return pathSegments
                .get(lastIndex)
                .getEndXMM();
    }

    @Override
    public double getEndYMM() {

        int lastIndex =
                pathSegments.size() - 1;

        return pathSegments
                .get(lastIndex)
                .getEndYMM();
    }

    /**
     * Returns how many individual paths are contained in this
     * composite path.
     */
    public int getSegmentCount() {
        return pathSegments.size();
    }

    /**
     * Returns one individual path segment.
     */
    public DrivePath getSegment(
            int segmentIndex
    ) {

        if (segmentIndex < 0
                || segmentIndex
                >= pathSegments.size()) {

            throw new IndexOutOfBoundsException(
                    "Invalid path segment index: "
                            + segmentIndex
            );
        }

        return pathSegments.get(
                segmentIndex
        );
    }

    /**
     * Returns the global distance where a segment starts.
     */
    public double getSegmentStartDistanceMM(
            int segmentIndex
    ) {

        if (segmentIndex < 0
                || segmentIndex
                >= segmentStartDistanceMM.length) {

            throw new IndexOutOfBoundsException(
                    "Invalid path segment index: "
                            + segmentIndex
            );
        }

        return segmentStartDistanceMM[
                segmentIndex
                ];
    }
}
