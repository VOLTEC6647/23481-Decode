package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class PedroMirror {
    public static double mirrorHeading(double heading){
        return  180 - (heading - 90);
    }
    public static Pose mirrorPose(Pose pose) {
        // 1. Mirror X: Total width (144) - current X
        // Since center is 72, the formula is: 2 * 72 - x
        double newX = 144 - pose.getX();

        // 2. Y stays the same (no vertical flip)
        double newY = pose.getY();

        // 3. Mirror Heading: 180 degrees - heading (or PI - heading in radians)
        // This reflects the angle across the Y-axis (e.g., 100 -> 80)
        double newHeading = Math.PI - pose.getHeading();

        // Optional: Normalize to -PI...PI range to keep numbers clean
        while (newHeading > Math.PI) newHeading -= 2 * Math.PI;
        while (newHeading <= -Math.PI) newHeading += 2 * Math.PI;

        return new Pose(newX, newY, newHeading);
    }
}
