package se.oru.coordination.coordination_oru.multirobotplanning.tests.dimpc;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.multirobotplanning.dimpc.MissionDiMOpt;
import se.oru.coordination.coordination_oru.multirobotplanning.dimpc.RecedingHorizonDiMOpt;


public class dimopt {

    public static void main(String[] args) {
        int R = 2;
        String file_map = "map1.png";
        // Robot Footprint Rectangle
        Coordinate corner1 = new Coordinate(-0.5,0.3);
        Coordinate corner2 = new Coordinate(0.5,0.3);
        Coordinate corner3 = new Coordinate(0.5,-0.3);
        Coordinate corner4 = new Coordinate(-0.5,-0.3);
        Coordinate[] footprint = {
                corner1, corner2, corner3, corner4
        };
        // Start and Goal
        Pose s1 = new Pose( 1.0, 1.0,0.5 *Math.PI);
        Pose s2 = new Pose(10.0, 1.0,0.5 *Math.PI);
        Pose g1 = new Pose(10.0,10.0,0.0);
        Pose g2 = new Pose( 1.0,10.0,0.0);

        Pose [] mrStart = { s1, s2 };
        Pose [] mrGoal = { g1, g2 };

        // Multirobot solver
        RecedingHorizonDiMOpt dimopt = new RecedingHorizonDiMOpt(R, 1.0,1.0, file_map);
        dimopt.setupTrajectoryEnvelopeCoordinator();
        dimopt.setMultirobotProblem(mrStart, mrGoal);
        dimopt.setFootprintEqual(footprint);





        MissionDiMOpt mission = new MissionDiMOpt(R);
        MissionDiMOpt.SE2[] start = {
                new MissionDiMOpt.SE2(1,0,0),
                new MissionDiMOpt.SE2(0,3,0)
        };
        MissionDiMOpt.SE2[] goal = {
                new MissionDiMOpt.SE2(0,2,0),
                new MissionDiMOpt.SE2(0,0,2)
        };
        MissionDiMOpt.Polytope.Halfspace[] hps = {
                new MissionDiMOpt.Polytope.Halfspace(4,0,0),
                new MissionDiMOpt.Polytope.Halfspace(0,5,0)
        };

        mission.setMultirobotStart(start)
                .setMultirobotGoal(goal)
                .setSamePolytopeFreeSpace(new MissionDiMOpt.Polytope(hps));

    }

}
