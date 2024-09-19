//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name = "ABMSSA Pure Pursuit", group = "Autonomous")
//public class MainOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        SalpSwarmAlgorithm.Position startPos = new SalpSwarmAlgorithm.Position(0, 0);
//        SalpSwarmAlgorithm.Position targetPos = new SalpSwarmAlgorithm.Position(10, 10);
//        List<SalpSwarmAlgorithm.Position> obstacles = new ArrayList<>();
//        obstacles.add(new SalpSwarmAlgorithm.Position(5, 5));
//
//        SalpSwarmAlgorithm salpSwarmAlgorithm = new SalpSwarmAlgorithm(startPos, targetPos, obstacles);
//        salpSwarmAlgorithm.optimize();
//        List<SalpSwarmAlgorithm.Position> optimizedPath = salpSwarmAlgorithm.getBestPath();
//
//        PurePursuit purePursuit = new PurePursuit(convertPath(optimizedPath));
//        waitForStart();
//        purePursuit.followPath();
//    }
//
//    public List<PurePursuit.Waypoint> convertPath(List<SalpSwarmAlgorithm.Position> optimizedPath) {
//        List<PurePursuit.Waypoint> path = new ArrayList<>();
//        for (int i = 0; i < optimizedPath.size(); i++) {
//            SalpSwarmAlgorithm.Position pos = optimizedPath.get(i);
//            double heading = 0;
//            if (i < optimizedPath.size() - 1) {
//                SalpSwarmAlgorithm.Position nextPos = optimizedPath.get(i + 1);
//                heading = calculateHeading(pos, nextPos);
//            }
//            path.add(new PurePursuit.Waypoint(pos.x, pos.y, heading));
//        }
//        return path;
//    }
//
//    private double calculateHeading(SalpSwarmAlgorithm.Position current, SalpSwarmAlgorithm.Position next) {
//        // Calculate the heading between the current and next position
//        return Math.atan2(next.y - current.y, next.x - current.x);
//    }
//}
