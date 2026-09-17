package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;

/**
 * High-performance field visualization and geometry renderer for the Biobuzz season.
 * Integrates with FTControl Panels to draw polygonal zones, game pieces, and autonomous paths.
 *
 * <p>Designed for zero latency impact: static elements and trajectories are rendered
 * during {@code init_loop()}, leaving the match loop completely unburdened.
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class BiobuzzFieldRenderer {

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    public static final Style STYLE_BLUE = new Style("#1E88E5", "#0D47A1", 1.5);
    public static final Style STYLE_RED = new Style("#E53935", "#B71C1C", 1.5);
    public static final Style STYLE_INTAKE_ZONE = new Style("#FFF9C4", "#FBC02D", 1.5);
    public static final Style STYLE_SCORE_ZONE = new Style("#C8E6C9", "#43A047", 1.5);
    public static final Style STYLE_OBSTACLE = new Style("#EEEEEE", "#616161", 2.0);
    public static final Style STYLE_PATH_LINE = new Style("", "#00E676", 2.0);
    public static final Style STYLE_WAYPOINT = new Style("#FFEB3B", "#E65100", 1.0);

    private BiobuzzFieldRenderer() {}

    /**
     * Initializes the field coordinate offsets for Pedro Pathing.
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Draws a rectangular zone on the field (e.g., intake zones, scoring basins, observation areas).
     */
    public static void drawRectZone(double x1, double y1, double x2, double y2, Style style) {
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y1);
        panelsField.line(x2, y2);
        panelsField.line(x1, y2);
        panelsField.line(x1, y1);
    }

    /**
     * Draws an arbitrary closed polygon with N vertices.
     * Coordinate pairs are passed sequentially: x1, y1, x2, y2, ..., xN, yN.
     */
    public static void drawPolygon(Style style, double... xyPairs) {
        if (xyPairs == null || xyPairs.length < 4 || xyPairs.length % 2 != 0) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(xyPairs[0], xyPairs[1]);

        for (int i = 2; i < xyPairs.length; i += 2) {
            panelsField.line(xyPairs[i], xyPairs[i + 1]);
        }

        // Close polygon by drawing line back to first vertex
        panelsField.line(xyPairs[0], xyPairs[1]);
    }

    /**
     * Draws a Polygon2d object.
     */
    public static void drawPolygon(Polygon2d polygon, Style style) {
        if (polygon == null || polygon.getVertices() == null || polygon.getVertices().length < 2) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(polygon.getVertices()[0].getX(), polygon.getVertices()[0].getY());

        for (int i = 1; i < polygon.getVertices().length; i++) {
            panelsField.line(polygon.getVertices()[i].getX(), polygon.getVertices()[i].getY());
        }

        panelsField.line(polygon.getVertices()[0].getX(), polygon.getVertices()[0].getY());
    }

    /**
     * Draws an oriented game piece (sample / element) at a specific Pose.
     */
    public static void drawGamePiece(Pose pose, double width, double length, Style style) {
        if (pose == null) return;

        double halfW = width / 2.0;
        double halfL = length / 2.0;
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());

        double[][] corners = {
                {-halfL, -halfW},
                { halfL, -halfW},
                { halfL,  halfW},
                {-halfL,  halfW}
        };

        double[] transformed = new double[8];
        for (int i = 0; i < 4; i++) {
            transformed[i * 2]     = pose.x() + (corners[i][0] * cos - corners[i][1] * sin);
            transformed[i * 2 + 1] = pose.y() + (corners[i][0] * sin + corners[i][1] * cos);
        }

        drawPolygon(style, transformed);
    }

    /**
     * Draws a waypoint marker (small circle).
     */
    public static void drawWaypoint(Pose pose, Style style) {
        if (pose == null) return;
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(2.5);
    }

    /**
     * Draws a complete preview of the Biobuzz autonomous trajectory and waypoints.
     */
    public static void drawAutoPreview(AllianceEnum alliance) {
        DataStorage.alliance = alliance;

        Pose start = BiobuzzAutoPaths.getStartPose();
        Pose preload = BiobuzzAutoPaths.getPreloadScore();
        Pose control = BiobuzzAutoPaths.CYCLE_1_INTAKE_CONTROL.getPose();
        Pose intake = BiobuzzAutoPaths.getCycle1Intake();
        Pose score = BiobuzzAutoPaths.getCycle1Score();
        Pose park = BiobuzzAutoPaths.getParkPose();

        Style allianceStyle = (alliance == AllianceEnum.Red) ? STYLE_RED : STYLE_BLUE;

        // Draw Waypoint Markers
        drawWaypoint(start, allianceStyle);
        drawWaypoint(preload, STYLE_SCORE_ZONE);
        drawWaypoint(intake, STYLE_INTAKE_ZONE);
        drawWaypoint(score, STYLE_SCORE_ZONE);
        drawWaypoint(park, allianceStyle);

        // Draw Straight and Curved Lines connecting waypoints
        panelsField.setStyle(STYLE_PATH_LINE);

        // Start -> Preload
        panelsField.moveCursor(start.x(), start.y());
        panelsField.line(preload.x(), preload.y());

        // Preload -> Intake (Quadratic Bezier preview using 10 segments)
        double prevX = preload.x();
        double prevY = preload.y();
        for (int i = 1; i <= 10; i++) {
            double t = i / 10.0;
            double u = 1.0 - t;
            double bx = u * u * preload.x() + 2 * u * t * control.x() + t * t * intake.x();
            double by = u * u * preload.y() + 2 * u * t * control.y() + t * t * intake.y();
            panelsField.moveCursor(prevX, prevY);
            panelsField.line(bx, by);
            prevX = bx;
            prevY = by;
        }

        // Intake -> Score
        panelsField.moveCursor(intake.x(), intake.y());
        panelsField.line(score.x(), score.y());

        // Score -> Park
        panelsField.moveCursor(score.x(), score.y());
        panelsField.line(park.x(), park.y());

        panelsField.update();
    }
}
