package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.tuning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Generates a self-contained static HTML page for drivetrain
 * feedforward tuning results.
 *
 * Plot:
 *      X-axis = absolute encoder velocity in ticks/second
 *      Y-axis = absolute commanded motor power
 *
 * Fitted equation:
 *      absolutePower = kS + kV * absoluteVelocity
 *
 * The generated page does not require internet access or any
 * external JavaScript/chart library.
 */
public final class FeedforwardPlotGenerator {

    private FeedforwardPlotGenerator() {
        /*
         * Utility class: do not create instances.
         */
    }

    /**
     * Creates an HTML report containing the scatter plot,
     * fitted line, coefficient estimates, and measured data table.
     *
     * @param outputFile file where the HTML page will be saved
     * @param motorPowers recorded motor powers
     * @param velocitiesTicksPerSecond recorded encoder velocities
     * @param resultCount number of valid values in the arrays
     * @return calculated feedforward fit
     */
    public static FeedforwardFit generateHtmlReport(
            File outputFile,
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount
    ) throws IOException {

        validateInputs(
                outputFile,
                motorPowers,
                velocitiesTicksPerSecond,
                resultCount
        );

        FeedforwardFit fit = calculateFit(
                motorPowers,
                velocitiesTicksPerSecond,
                resultCount
        );

        File parentFolder = outputFile.getParentFile();

        if (parentFolder != null
                && !parentFolder.exists()
                && !parentFolder.mkdirs()) {

            throw new IOException(
                    "Unable to create report folder: "
                            + parentFolder.getAbsolutePath()
            );
        }

        String html = buildHtml(
                motorPowers,
                velocitiesTicksPerSecond,
                resultCount,
                fit
        );

        try (FileWriter writer =
                     new FileWriter(outputFile, false)) {

            writer.write(html);
        }

        return fit;
    }

    private static String buildHtml(
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount,
            FeedforwardFit fit
    ) {
        final int svgWidth = 920;
        final int svgHeight = 560;

        final int marginLeft = 90;
        final int marginRight = 40;
        final int marginTop = 45;
        final int marginBottom = 80;

        final int plotWidth =
                svgWidth - marginLeft - marginRight;

        final int plotHeight =
                svgHeight - marginTop - marginBottom;

        double maximumVelocity = 1.0;
        double maximumPower = 1.0;

        for (int index = 0;
             index < resultCount;
             index++) {

            maximumVelocity = Math.max(
                    maximumVelocity,
                    Math.abs(
                            velocitiesTicksPerSecond[index]
                    )
            );

            maximumPower = Math.max(
                    maximumPower,
                    Math.abs(motorPowers[index])
            );
        }

        /*
         * Add graph padding so the highest point is not located
         * directly on the border.
         */
        maximumVelocity *= 1.10;
        maximumPower =
                Math.min(
                        1.05,
                        maximumPower * 1.10
                );

        StringBuilder html = new StringBuilder();

        html.append("<!DOCTYPE html>\n");
        html.append("<html lang=\"en\">\n");
        html.append("<head>\n");
        html.append("<meta charset=\"UTF-8\">\n");
        html.append(
                "<meta name=\"viewport\" "
                        + "content=\"width=device-width, "
                        + "initial-scale=1.0\">\n"
        );

        html.append(
                "<title>FTC Drive Feedforward Tuning</title>\n"
        );

        appendCss(html);

        html.append("</head>\n");
        html.append("<body>\n");
        html.append("<main class=\"page\">\n");

        html.append(
                "<h1>FTC Drive Feedforward Tuning</h1>\n"
        );

        html.append(
                "<p class=\"subtitle\">"
                        + "Static drivetrain power versus encoder "
                        + "velocity report"
                        + "</p>\n"
        );

        appendSummaryCards(html, fit);

        html.append("<section class=\"card\">\n");
        html.append(
                "<h2>Motor power versus encoder velocity</h2>\n"
        );

        html.append(
                "<p class=\"description\">"
                        + "Points show measured steady-state values. "
                        + "The line represents the fitted equation "
                        + "<code>|power| = kS + kV × |velocity|</code>."
                        + "</p>\n"
        );

        html.append(
                "<svg viewBox=\"0 0 "
                        + svgWidth
                        + " "
                        + svgHeight
                        + "\" "
                        + "role=\"img\" "
                        + "aria-label=\"Feedforward tuning plot\">\n"
        );

        appendPlotBackground(
                html,
                marginLeft,
                marginTop,
                plotWidth,
                plotHeight
        );

        appendGridAndAxes(
                html,
                svgWidth,
                svgHeight,
                marginLeft,
                marginTop,
                marginBottom,
                plotWidth,
                plotHeight,
                maximumVelocity,
                maximumPower
        );

        appendFitLine(
                html,
                fit,
                marginLeft,
                marginTop,
                plotWidth,
                plotHeight,
                maximumVelocity,
                maximumPower
        );

        appendMeasuredPoints(
                html,
                motorPowers,
                velocitiesTicksPerSecond,
                resultCount,
                marginLeft,
                marginTop,
                plotWidth,
                plotHeight,
                maximumVelocity,
                maximumPower
        );

        appendAxisTitles(
                html,
                svgWidth,
                svgHeight,
                marginLeft,
                marginTop,
                marginBottom,
                plotHeight
        );

        html.append("</svg>\n");

        html.append(
                "<div class=\"legend\">"
                        + "<span><i class=\"point-symbol\"></i>"
                        + "Measured values</span>"
                        + "<span><i class=\"line-symbol\"></i>"
                        + "Linear fit</span>"
                        + "</div>\n"
        );

        html.append("</section>\n");

        appendResultTable(
                html,
                motorPowers,
                velocitiesTicksPerSecond,
                resultCount,
                fit
        );

        appendInstructions(html);

        html.append("</main>\n");
        html.append("</body>\n");
        html.append("</html>\n");

        return html.toString();
    }

    private static void appendCss(
            StringBuilder html
    ) {
        html.append("<style>\n");

        html.append(
                "*{box-sizing:border-box;}"
        );

        html.append(
                "body{"
                        + "margin:0;"
                        + "font-family:Arial,Helvetica,sans-serif;"
                        + "background:#f4f6f8;"
                        + "color:#17212b;"
                        + "}"
        );

        html.append(
                ".page{"
                        + "max-width:1100px;"
                        + "margin:0 auto;"
                        + "padding:32px 18px 60px;"
                        + "}"
        );

        html.append(
                "h1{"
                        + "margin:0;"
                        + "font-size:32px;"
                        + "}"
        );

        html.append(
                "h2{"
                        + "margin-top:0;"
                        + "font-size:21px;"
                        + "}"
        );

        html.append(
                ".subtitle{"
                        + "margin-top:7px;"
                        + "color:#5f6b76;"
                        + "}"
        );

        html.append(
                ".card{"
                        + "background:#ffffff;"
                        + "border:1px solid #dfe4e8;"
                        + "border-radius:12px;"
                        + "padding:22px;"
                        + "margin-top:20px;"
                        + "box-shadow:0 3px 10px rgba(0,0,0,.05);"
                        + "}"
        );

        html.append(
                ".summary-grid{"
                        + "display:grid;"
                        + "grid-template-columns:"
                        + "repeat(auto-fit,minmax(180px,1fr));"
                        + "gap:14px;"
                        + "margin-top:22px;"
                        + "}"
        );

        html.append(
                ".summary-card{"
                        + "background:#ffffff;"
                        + "border:1px solid #dfe4e8;"
                        + "border-radius:10px;"
                        + "padding:17px;"
                        + "}"
        );

        html.append(
                ".summary-label{"
                        + "font-size:13px;"
                        + "color:#68747f;"
                        + "text-transform:uppercase;"
                        + "letter-spacing:.05em;"
                        + "}"
        );

        html.append(
                ".summary-value{"
                        + "font-size:24px;"
                        + "font-weight:700;"
                        + "margin-top:7px;"
                        + "}"
        );

        html.append(
                ".description{"
                        + "color:#5b6670;"
                        + "line-height:1.5;"
                        + "}"
        );

        html.append(
                "svg{"
                        + "display:block;"
                        + "width:100%;"
                        + "height:auto;"
                        + "background:#ffffff;"
                        + "}"
        );

        html.append(
                ".legend{"
                        + "display:flex;"
                        + "gap:24px;"
                        + "justify-content:center;"
                        + "font-size:14px;"
                        + "margin-top:8px;"
                        + "}"
        );

        html.append(
                ".legend span{"
                        + "display:flex;"
                        + "align-items:center;"
                        + "gap:8px;"
                        + "}"
        );

        html.append(
                ".point-symbol{"
                        + "width:11px;"
                        + "height:11px;"
                        + "border-radius:50%;"
                        + "background:#1565c0;"
                        + "display:inline-block;"
                        + "}"
        );

        html.append(
                ".line-symbol{"
                        + "width:26px;"
                        + "height:3px;"
                        + "background:#d84315;"
                        + "display:inline-block;"
                        + "}"
        );

        html.append(
                "table{"
                        + "width:100%;"
                        + "border-collapse:collapse;"
                        + "margin-top:12px;"
                        + "}"
        );

        html.append(
                "th,td{"
                        + "text-align:right;"
                        + "padding:10px 12px;"
                        + "border-bottom:1px solid #e5e9ed;"
                        + "}"
        );

        html.append(
                "th:first-child,td:first-child{"
                        + "text-align:center;"
                        + "}"
        );

        html.append(
                "th{"
                        + "background:#f7f9fa;"
                        + "font-size:13px;"
                        + "}"
        );

        html.append(
                "code{"
                        + "background:#eef2f5;"
                        + "padding:2px 5px;"
                        + "border-radius:4px;"
                        + "}"
        );

        html.append(
                ".notes{"
                        + "line-height:1.65;"
                        + "}"
        );

        html.append("</style>\n");
    }

    private static void appendSummaryCards(
            StringBuilder html,
            FeedforwardFit fit
    ) {
        html.append("<section class=\"summary-grid\">\n");

        appendSummaryCard(
                html,
                "Estimated kS",
                format(fit.getKS(), 6)
        );

        appendSummaryCard(
                html,
                "Estimated kV",
                format(fit.getKV(), 9)
                        + " power/(tick/s)"
        );

        appendSummaryCard(
                html,
                "R²",
                format(fit.getRSquared(), 5)
        );

        appendSummaryCard(
                html,
                "Valid points",
                Integer.toString(fit.getValidPointCount())
        );

        html.append("</section>\n");
    }

    private static void appendSummaryCard(
            StringBuilder html,
            String label,
            String value
    ) {
        html.append("<div class=\"summary-card\">\n");

        html.append(
                "<div class=\"summary-label\">"
                        + escapeHtml(label)
                        + "</div>\n"
        );

        html.append(
                "<div class=\"summary-value\">"
                        + escapeHtml(value)
                        + "</div>\n"
        );

        html.append("</div>\n");
    }

    private static void appendPlotBackground(
            StringBuilder html,
            int marginLeft,
            int marginTop,
            int plotWidth,
            int plotHeight
    ) {
        html.append(
                "<rect x=\""
                        + marginLeft
                        + "\" y=\""
                        + marginTop
                        + "\" width=\""
                        + plotWidth
                        + "\" height=\""
                        + plotHeight
                        + "\" fill=\"#fbfcfd\" "
                        + "stroke=\"#cbd3da\"/>\n"
        );
    }

    private static void appendGridAndAxes(
            StringBuilder html,
            int svgWidth,
            int svgHeight,
            int marginLeft,
            int marginTop,
            int marginBottom,
            int plotWidth,
            int plotHeight,
            double maximumVelocity,
            double maximumPower
    ) {
        int divisions = 5;

        for (int index = 0;
             index <= divisions;
             index++) {

            double fraction =
                    (double) index / divisions;

            double x =
                    marginLeft
                            + fraction * plotWidth;

            double y =
                    marginTop
                            + plotHeight
                            - fraction * plotHeight;

            double velocityLabel =
                    fraction * maximumVelocity;

            double powerLabel =
                    fraction * maximumPower;

            html.append(
                    "<line x1=\""
                            + format(x, 2)
                            + "\" y1=\""
                            + marginTop
                            + "\" x2=\""
                            + format(x, 2)
                            + "\" y2=\""
                            + (marginTop + plotHeight)
                            + "\" stroke=\"#e2e7eb\"/>\n"
            );

            html.append(
                    "<line x1=\""
                            + marginLeft
                            + "\" y1=\""
                            + format(y, 2)
                            + "\" x2=\""
                            + (marginLeft + plotWidth)
                            + "\" y2=\""
                            + format(y, 2)
                            + "\" stroke=\"#e2e7eb\"/>\n"
            );

            html.append(
                    "<text x=\""
                            + format(x, 2)
                            + "\" y=\""
                            + (svgHeight
                            - marginBottom
                            + 25)
                            + "\" text-anchor=\"middle\" "
                            + "font-size=\"13\" fill=\"#56616b\">"
                            + format(velocityLabel, 0)
                            + "</text>\n"
            );

            html.append(
                    "<text x=\""
                            + (marginLeft - 14)
                            + "\" y=\""
                            + format(y + 4, 2)
                            + "\" text-anchor=\"end\" "
                            + "font-size=\"13\" fill=\"#56616b\">"
                            + format(powerLabel, 2)
                            + "</text>\n"
            );
        }

        html.append(
                "<line x1=\""
                        + marginLeft
                        + "\" y1=\""
                        + (marginTop + plotHeight)
                        + "\" x2=\""
                        + (marginLeft + plotWidth)
                        + "\" y2=\""
                        + (marginTop + plotHeight)
                        + "\" stroke=\"#18222c\" "
                        + "stroke-width=\"2\"/>\n"
        );

        html.append(
                "<line x1=\""
                        + marginLeft
                        + "\" y1=\""
                        + marginTop
                        + "\" x2=\""
                        + marginLeft
                        + "\" y2=\""
                        + (marginTop + plotHeight)
                        + "\" stroke=\"#18222c\" "
                        + "stroke-width=\"2\"/>\n"
        );
    }

    private static void appendFitLine(
            StringBuilder html,
            FeedforwardFit fit,
            int marginLeft,
            int marginTop,
            int plotWidth,
            int plotHeight,
            double maximumVelocity,
            double maximumPower
    ) {
        double startPower =
                Math.max(0.0, fit.getKS());

        double endPower =
                fit.getKS()
                        + fit.getKV()
                        * maximumVelocity;

        double x1 = marginLeft;

        double y1 =
                marginTop
                        + plotHeight
                        - startPower
                        / maximumPower
                        * plotHeight;

        double x2 =
                marginLeft + plotWidth;

        double y2 =
                marginTop
                        + plotHeight
                        - endPower
                        / maximumPower
                        * plotHeight;

        y1 = clamp(
                y1,
                marginTop,
                marginTop + plotHeight
        );

        y2 = clamp(
                y2,
                marginTop,
                marginTop + plotHeight
        );

        html.append(
                "<line x1=\""
                        + format(x1, 2)
                        + "\" y1=\""
                        + format(y1, 2)
                        + "\" x2=\""
                        + format(x2, 2)
                        + "\" y2=\""
                        + format(y2, 2)
                        + "\" stroke=\"#d84315\" "
                        + "stroke-width=\"3\"/>\n"
        );
    }

    private static void appendMeasuredPoints(
            StringBuilder html,
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount,
            int marginLeft,
            int marginTop,
            int plotWidth,
            int plotHeight,
            double maximumVelocity,
            double maximumPower
    ) {
        for (int index = 0;
             index < resultCount;
             index++) {

            double velocity =
                    Math.abs(
                            velocitiesTicksPerSecond[index]
                    );

            double power =
                    Math.abs(motorPowers[index]);

            double x =
                    marginLeft
                            + velocity
                            / maximumVelocity
                            * plotWidth;

            double y =
                    marginTop
                            + plotHeight
                            - power
                            / maximumPower
                            * plotHeight;

            html.append(
                    "<circle cx=\""
                            + format(x, 2)
                            + "\" cy=\""
                            + format(y, 2)
                            + "\" r=\"6\" "
                            + "fill=\"#1565c0\" "
                            + "stroke=\"#ffffff\" "
                            + "stroke-width=\"2\">"
            );

            html.append(
                    "<title>Power: "
                            + format(
                            motorPowers[index],
                            3
                    )
                            + ", Velocity: "
                            + format(
                            velocitiesTicksPerSecond[index],
                            1
                    )
                            + " ticks/s</title>"
            );

            html.append("</circle>\n");
        }
    }

    private static void appendAxisTitles(
            StringBuilder html,
            int svgWidth,
            int svgHeight,
            int marginLeft,
            int marginTop,
            int marginBottom,
            int plotHeight
    ) {
        html.append(
                "<text x=\""
                        + (svgWidth / 2)
                        + "\" y=\""
                        + (svgHeight - 18)
                        + "\" text-anchor=\"middle\" "
                        + "font-size=\"16\" "
                        + "font-weight=\"600\">"
                        + "Absolute encoder velocity "
                        + "(ticks/second)"
                        + "</text>\n"
        );

        double yCenter =
                marginTop + plotHeight / 2.0;

        html.append(
                "<text x=\"22\" y=\""
                        + format(yCenter, 2)
                        + "\" text-anchor=\"middle\" "
                        + "font-size=\"16\" "
                        + "font-weight=\"600\" "
                        + "transform=\"rotate(-90 22 "
                        + format(yCenter, 2)
                        + ")\">"
                        + "Absolute motor power"
                        + "</text>\n"
        );
    }

    private static void appendResultTable(
            StringBuilder html,
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount,
            FeedforwardFit fit
    ) {
        html.append("<section class=\"card\">\n");
        html.append("<h2>Measured results</h2>\n");

        html.append("<table>\n");
        html.append("<thead><tr>");
        html.append("<th>Point</th>");
        html.append("<th>Motor power</th>");
        html.append("<th>Velocity (ticks/s)</th>");
        html.append("<th>|Power|</th>");
        html.append("<th>|Velocity|</th>");
        html.append("<th>Fitted power</th>");
        html.append("<th>Residual</th>");
        html.append("</tr></thead>\n");
        html.append("<tbody>\n");

        for (int index = 0;
             index < resultCount;
             index++) {

            double absoluteVelocity =
                    Math.abs(
                            velocitiesTicksPerSecond[index]
                    );

            double absolutePower =
                    Math.abs(motorPowers[index]);

            double fittedPower =
                    fit.getKS()
                            + fit.getKV()
                            * absoluteVelocity;

            double residual =
                    absolutePower - fittedPower;

            html.append("<tr>");

            appendTableCell(
                    html,
                    Integer.toString(index + 1)
            );

            appendTableCell(
                    html,
                    format(motorPowers[index], 3)
            );

            appendTableCell(
                    html,
                    format(
                            velocitiesTicksPerSecond[index],
                            1
                    )
            );

            appendTableCell(
                    html,
                    format(absolutePower, 3)
            );

            appendTableCell(
                    html,
                    format(absoluteVelocity, 1)
            );

            appendTableCell(
                    html,
                    format(fittedPower, 4)
            );

            appendTableCell(
                    html,
                    format(residual, 4)
            );

            html.append("</tr>\n");
        }

        html.append("</tbody>\n");
        html.append("</table>\n");
        html.append("</section>\n");
    }

    private static void appendTableCell(
            StringBuilder html,
            String value
    ) {
        html.append(
                "<td>"
                        + escapeHtml(value)
                        + "</td>"
        );
    }

    private static void appendInstructions(
            StringBuilder html
    ) {
        html.append(
                "<section class=\"card notes\">\n"
        );

        html.append("<h2>Using the fitted values</h2>\n");

        html.append(
                "<p>The fitted equation is:</p>\n"
        );

        html.append(
                "<p><code>"
                        + "power = kS × sign(velocity) "
                        + "+ kV × velocity"
                        + "</code></p>\n"
        );

        html.append(
                "<p>If your controller uses velocity in "
                        + "<strong>ticks/second</strong>, use the "
                        + "reported kV directly.</p>\n"
        );

        html.append(
                "<p>If your controller uses velocity in "
                        + "<strong>mm/second</strong>, convert it with:"
                        + "</p>\n"
        );

        html.append(
                "<p><code>"
                        + "kV_mm = kV_ticks × ticksPerMM"
                        + "</code></p>\n"
        );

        html.append(
                "<p>Keep kA at zero initially. A steady-state "
                        + "power sweep identifies kS and kV, but does "
                        + "not provide a reliable acceleration "
                        + "feedforward value.</p>\n"
        );

        html.append("</section>\n");
    }

    /**
     * Fits:
     *
     *      |power| = kS + kV * |velocity|
     */
    public static FeedforwardFit calculateFit(
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount
    ) {
        double sumVelocity = 0.0;
        double sumPower = 0.0;
        double sumVelocitySquared = 0.0;
        double sumVelocityPower = 0.0;

        int validPointCount = 0;

        for (int index = 0;
             index < resultCount;
             index++) {

            double velocity =
                    Math.abs(
                            velocitiesTicksPerSecond[index]
                    );

            double power =
                    Math.abs(motorPowers[index]);

            if (!Double.isFinite(velocity)
                    || !Double.isFinite(power)
                    || velocity < 1.0) {

                continue;
            }

            sumVelocity += velocity;
            sumPower += power;

            sumVelocitySquared +=
                    velocity * velocity;

            sumVelocityPower +=
                    velocity * power;

            validPointCount++;
        }

        if (validPointCount < 2) {
            return new FeedforwardFit(
                    0.0,
                    0.0,
                    0.0,
                    validPointCount
            );
        }

        double denominator =
                validPointCount
                        * sumVelocitySquared
                        - sumVelocity
                        * sumVelocity;

        if (Math.abs(denominator) < 1e-12) {
            return new FeedforwardFit(
                    0.0,
                    0.0,
                    0.0,
                    validPointCount
            );
        }

        double kV =
                (
                        validPointCount
                                * sumVelocityPower
                                - sumVelocity
                                * sumPower
                ) / denominator;

        double kS =
                (
                        sumPower
                                - kV
                                * sumVelocity
                ) / validPointCount;

        double meanPower =
                sumPower / validPointCount;

        double totalVariation = 0.0;
        double residualVariation = 0.0;

        for (int index = 0;
             index < resultCount;
             index++) {

            double velocity =
                    Math.abs(
                            velocitiesTicksPerSecond[index]
                    );

            double power =
                    Math.abs(motorPowers[index]);

            if (!Double.isFinite(velocity)
                    || !Double.isFinite(power)
                    || velocity < 1.0) {

                continue;
            }

            double predictedPower =
                    kS + kV * velocity;

            double totalDifference =
                    power - meanPower;

            double residual =
                    power - predictedPower;

            totalVariation +=
                    totalDifference * totalDifference;

            residualVariation +=
                    residual * residual;
        }

        double rSquared = 0.0;

        if (totalVariation > 1e-12) {
            rSquared =
                    1.0
                            - residualVariation
                            / totalVariation;
        }

        return new FeedforwardFit(
                kS,
                kV,
                rSquared,
                validPointCount
        );
    }

    private static void validateInputs(
            File outputFile,
            double[] motorPowers,
            double[] velocitiesTicksPerSecond,
            int resultCount
    ) {
        if (outputFile == null) {
            throw new IllegalArgumentException(
                    "Output file cannot be null."
            );
        }

        if (motorPowers == null
                || velocitiesTicksPerSecond == null) {

            throw new IllegalArgumentException(
                    "Result arrays cannot be null."
            );
        }

        if (resultCount < 0
                || resultCount > motorPowers.length
                || resultCount
                > velocitiesTicksPerSecond.length) {

            throw new IllegalArgumentException(
                    "Invalid result count."
            );
        }
    }

    private static String format(
            double value,
            int decimalPlaces
    ) {
        return String.format(
                Locale.US,
                "%."
                        + decimalPlaces
                        + "f",
                value
        );
    }

    private static String escapeHtml(
            String value
    ) {
        return value
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
                .replace("\"", "&quot;");
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    /**
     * Result of the feedforward linear regression.
     */
    public static final class FeedforwardFit {

        private final double kS;
        private final double kV;
        private final double rSquared;
        private final int validPointCount;

        private FeedforwardFit(
                double kS,
                double kV,
                double rSquared,
                int validPointCount
        ) {
            this.kS = kS;
            this.kV = kV;
            this.rSquared = rSquared;
            this.validPointCount = validPointCount;
        }

        public double getKS() {
            return kS;
        }

        public double getKV() {
            return kV;
        }

        public double getRSquared() {
            return rSquared;
        }

        public int getValidPointCount() {
            return validPointCount;
        }
    }
}