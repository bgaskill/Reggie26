package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathplannerUTILS {
    public static PathPlannerPath createLLPath(Pose2d botPose, Pose2d tagPose){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(botPose, tagPose);
        PathConstraints constraints = new PathConstraints(3.0,3.0, 2*Math.PI, 4*Math.PI);

        PathPlannerPath path = new PathPlannerPath(
        waypoints,
         constraints, 
        null, 
        new GoalEndState(0.0, tagPose.getRotation()));

        path.preventFlipping = true;

        return path;
    }
}
