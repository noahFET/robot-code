package frc.robot.questOdometry;

public class MathUtils {
    public static double[] rotateVector2(double x, double y, double radians){
        //double r = Math.toRadians(degrees);
        double rotatedX = x*Math.cos(radians)-y*Math.sin(radians);
        double rotatedY = y*Math.cos(radians)+x*Math.sin(radians);
        return(new double[]{rotatedX,rotatedY});
    }
}
