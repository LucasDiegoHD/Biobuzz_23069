package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * 2D closed polygon representation defined by ordered vertices.
 * Supports native Pedro Pathing Poses with ray-casting containment detection.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class Polygon2d {

    /**
     * Represents an immutable 2D Cartesian coordinate.
     */
    public static class Point {
        private final double x;
        private final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }

    private final Point[] vertices;

    public Polygon2d(Point... vertices) {
        this.vertices = (vertices != null) ? vertices : new Point[0];
    }

    public Polygon2d(Pose... poses) {
        if (poses != null) {
            this.vertices = new Point[poses.length];
            for (int i = 0; i < poses.length; i++) {
                this.vertices[i] = new Point(poses[i].x(), poses[i].y());
            }
        } else {
            this.vertices = new Point[0];
        }
    }

    public Point[] getVertices() {
        return vertices;
    }

    /**
     * Determines whether a coordinate pair is inside this polygon using ray-casting.
     */
    public boolean contains(double x, double y) {
        if (vertices.length < 3) return false;

        boolean inside = false;
        int j = vertices.length - 1;

        for (int i = 0; i < vertices.length; i++) {
            double xi = vertices[i].getX();
            double yi = vertices[i].getY();
            double xj = vertices[j].getX();
            double yj = vertices[j].getY();

            boolean intersect = ((yi > y) != (yj > y))
                    && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
            j = i;
        }

        return inside;
    }

    public boolean contains(Point point) {
        return point != null && contains(point.getX(), point.getY());
    }

    public boolean contains(Pose pose) {
        return pose != null && contains(pose.x(), pose.y());
    }
}
