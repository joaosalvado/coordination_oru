package se.oru.coordination.coordination_oru.multirobotplanning.tests.dimpc;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.vividsolutions.jts.algorithm.MinimumBoundingCircle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.multirobotplanning.dimpc.MissionDiMOpt;
import se.oru.coordination.coordination_oru.multirobotplanning.dimpc.RecedingHorizonDiMOpt;

import java.io.PrintWriter;
import java.util.ArrayList;


public class dimopt {

    public static void main(String[] args) {
        int R = 2;
        String file_map = "map-empty.yaml";
        // Robot Footprint Rectangle
        Coordinate corner1 = new Coordinate(-0.2,0.1);
        Coordinate corner2 = new Coordinate(0.2,0.1);
        Coordinate corner3 = new Coordinate(0.2,-0.1);
        Coordinate corner4 = new Coordinate(-0.2,-0.1);
        Coordinate[] footprint = {
                corner1, corner2, corner3, corner4
        };

        // Start and Goal
        Pose s1 = new Pose( 1.0, 1.0,Math.PI);
        Pose s2 = new Pose(8.0, 1.0,0.5 *Math.PI);
        Pose s3 = new Pose(4.0, 8.0,-0.5 *Math.PI);
        Pose s4 = new Pose(1.0, 4.0, 0*Math.PI);
        Pose g1 = new Pose(3.0,7.0,0.0);
        Pose g2 = new Pose( 1.0,8.0,-0.5*Math.PI);
        Pose g3 = new Pose( 4.0,1.0,0.0);
        Pose g4 = new Pose(8.0, 4.0, 0*Math.PI);

        //Pose [] mrStart = { s1, s2, s3, s4, g1, g2, g3, g4 };
        //Pose [] mrGoal = { g1, g2, g3, g4, s1, s2, s3, s4 };

         Pose [] mrStart = { g1, g2};
         Pose [] mrGoal = { s1, s2};


        // Initialize Multi-robot solver
        RecedingHorizonDiMOpt dimopt = new RecedingHorizonDiMOpt(R, 1.0,1.0, file_map);
        dimopt.setN(20); // discretization / horizon
        // Set robots footprint to be all the same
        dimopt.setFootprintEqual(footprint);
        // Setup trajectory envelope
        // Note one can use dimopt.getTrajectoryEnvelopeCoordinator() and setup it up
        dimopt.setupTrajectoryEnvelopeCoordinator(mrStart);

        // Set Multi-robot start and goal pose
        dimopt.addMultirobotProblem(mrStart, mrGoal);
        // Solves the given problem by dispatching multi-robot mission on
        // a receding horizon manner to the trajectory envelope coordinator
        dimopt.solve();

        // New multi-robot problem, sending robots to start pose
        // dimopt.addMultirobotProblem(mrStart);
        // dimopt.solve();
    }

}
