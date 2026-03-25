package frc.autostuff;

public class AutoHelpers {
    // I vibe coded ts
    public class ArcUtils {

        public static class Point {
            double x, y;
            public Point(double x, double y) { this.x = x; this.y = y; }
        }

        /**
         * Finds the point on an arc closest to a target point.
         * @param center The center of the circle the arc belongs to.
         * @param radius The radius of the circle.
         * @param startAngle The starting angle of the arc in radians.
         * @param endAngle The ending angle of the arc in radians (must be > startAngle).
         * @param target The external point.
         * @return The closest Point on the arc.
         */
        public static Point findClosestPointOnArc(Point center, double radius, 
                                                double startAngle, double endAngle, 
                                                Point target) {
            // 1. Calculate the angle of the target point relative to center
            double dx = target.x - center.x;
            double dy = target.y - center.y;
            double targetAngle = Math.atan2(dy, dx);

            // 2. Normalize angle to handle the 2*PI wrap-around
            double normalizedTarget = normalizeAngle(targetAngle, startAngle);

            // 3. Determine if projection is within arc span
            if (normalizedTarget >= startAngle && normalizedTarget <= endAngle) {
                return new Point(
                    center.x + radius * Math.cos(targetAngle),
                    center.y + radius * Math.sin(targetAngle)
                );
            } else {
                // 4. Outside arc; return the closer of the two endpoints
                Point pStart = new Point(center.x + radius * Math.cos(startAngle), 
                                        center.y + radius * Math.sin(startAngle));
                Point pEnd = new Point(center.x + radius * Math.cos(endAngle), 
                                    center.y + radius * Math.sin(endAngle));

                double distSqStart = Math.pow(target.x - pStart.x, 2) + Math.pow(target.y - pStart.y, 2);
                double distSqEnd = Math.pow(target.x - pEnd.x, 2) + Math.pow(target.y - pEnd.y, 2);

                return (distSqStart < distSqEnd) ? pStart : pEnd;
            }
        }

        private static double normalizeAngle(double angle, double reference) {
            double twoPi = 2 * Math.PI;
            while (angle < reference) angle += twoPi;
            while (angle >= reference + twoPi) angle -= twoPi;
            return angle;
        }
    }
}
