package org.firstinspires.ftc.teamcode.utility.FTCDashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double CIRCLE_ROBOT_RADIUS = 9; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), CIRCLE_ROBOT_RADIUS);
        drawHeadingLine(canvas, pose, CIRCLE_ROBOT_RADIUS);
    }

    public static void drawHeadingLine(Canvas canvas, Pose2d pose, double length){
        Vector2d v = pose.headingVec().times(length);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRectRobot(Canvas canvas, Pose2d centerPose, double width, double height){
        double halfHeight = height/2;
        double halfWidth = width/2;
        double centerToCornerDist = Math.sqrt( (halfWidth*halfWidth) + (halfHeight*halfHeight) );
        Vector2d centerXY = new Vector2d(centerPose.getX(), centerPose.getY());

        drawHeadingLine(canvas, centerPose, halfHeight);

        Vector2d corner1 = centerXY.plus(Vector2d.polar( centerToCornerDist, centerPose.getHeading() + Math.atan(halfWidth / halfHeight) ));
        Vector2d corner2 = centerXY.plus(Vector2d.polar( centerToCornerDist, centerPose.getHeading() + Math.atan(halfHeight / halfWidth) + Math.PI/2));
        Vector2d corner3 = centerXY.plus(Vector2d.polar( centerToCornerDist, centerPose.getHeading() + Math.atan(halfWidth / halfHeight) + Math.PI ));
        Vector2d corner4 = centerXY.plus(Vector2d.polar( centerToCornerDist, centerPose.getHeading() + Math.atan(halfHeight / halfWidth) + (3*Math.PI)/2 ));

        drawLineBetween(canvas, corner1, corner2);
        drawLineBetween(canvas, corner2, corner3);
        drawLineBetween(canvas, corner3, corner4);
        drawLineBetween(canvas, corner4, corner1);
    }

    public static void drawLineBetween(Canvas canvas, Vector2d point1, Vector2d point2){
        canvas.strokeLine(point1.getX(), point1.getY(), point2.getX(), point2.getY());
    }
}