package frc.robot.util;

public class LimelightTools {
    public static double distFromTower(double angle){ //returns horizontal distance in meters, angle in radians
        final double HEIGHT = 2.6416;
        final double LL_HEIGHT = 0.7;
        double heightDiff = HEIGHT - LL_HEIGHT;
        angle += Math.PI * 50 / 180;
        // return Math.sin((0.25 * Math.PI) - angle) * heightDiff / Math.sin(angle); //Law of sines
        return heightDiff / Math.tan(angle); //SOH CAH TOA
    }
}
