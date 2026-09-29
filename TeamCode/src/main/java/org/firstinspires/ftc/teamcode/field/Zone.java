package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

/*
 * A Zone is a rectangle on the field, lined up with the field walls.
 * All numbers are in inches, using Pedro Pathing field coordinates (0-144).
 *
 * It answers questions like:
 *   - "Is this point inside the zone?"             -> contains(x, y)
 *   - "Would a round robot here touch the zone?"   -> overlapsCircle(x, y, radius)
 *   - "Do these two zones overlap?"                -> overlaps(otherZone)
 *
 * Zones can't be changed after they're made. Methods like expandedBy() hand
 * back a NEW zone instead of changing this one, so nobody can accidentally
 * move the hive for everyone else.
 */
public class Zone {

    public final String name;
    public final double minX, maxX, minY, maxY;

    public Zone(String name, double minX, double maxX, double minY, double maxY) {
        if (minX > maxX || minY > maxY) {
            throw new IllegalArgumentException("Zone " + name + ": min must be <= max");
        }
        this.name = name;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    public double centerX() { return (minX + maxX) / 2; }
    public double centerY() { return (minY + maxY) / 2; }
    public double width()   { return maxX - minX; }
    public double height()  { return maxY - minY; }

    // Is the point (x, y) inside this zone? Points exactly on the edge count as inside.
    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    public boolean contains(Pose pose) {
        return contains(pose.getX(), pose.getY());
    }

    // Would a circle centered at (x, y) touch this zone?
    // Trick: find the point in the zone closest to the circle's center,
    // then check if that point is within one radius of the center.
    public boolean overlapsCircle(double x, double y, double radius) {
        double closestX = Math.max(minX, Math.min(x, maxX));
        double closestY = Math.max(minY, Math.min(y, maxY));
        double dx = x - closestX;
        double dy = y - closestY;
        return dx * dx + dy * dy <= radius * radius;
    }

    public boolean overlaps(Zone other) {
        return minX <= other.maxX && maxX >= other.minX
            && minY <= other.maxY && maxY >= other.minY;
    }

    // A new zone that is bigger by 'margin' inches on every side.
    // Handy for "keep the robot's CENTER out of here" checks.
    public Zone expandedBy(double margin) {
        return new Zone(name, minX - margin, maxX + margin, minY - margin, maxY + margin);
    }

    // The matching zone for the other alliance. The BIOBUZZ field looks the same
    // when spun 180 degrees around its center, so (x, y) becomes (144 - x, 144 - y).
    public Zone rotated180(String newName) {
        double size = FieldConstants.FIELD_SIZE;
        return new Zone(newName, size - maxX, size - minX, size - maxY, size - minY);
    }

    @Override
    public String toString() {
        return String.format("%s[x %.1f-%.1f, y %.1f-%.1f]", name, minX, maxX, minY, maxY);
    }
}
