package se.oru.coordination.coordination_oru.multirobotoptimization.tests.dimopt;

import com.google.common.collect.ObjectArrays;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.util.Comparator;
import java.util.HashSet;
import java.util.Arrays;


import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;

public class test1 {

    public static void main(String[] args) {
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // Setup
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        double MAX_ACCEL = 1.0;
        double MAX_VEL = 1.0;
        //Instantiate a trajectory envelope coordinator.
        //The TrajectoryEnvelopeCoordinatorSimulation implementation provides
        // -- the factory method getNewTracker() which returns a trajectory envelope tracker
        // -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
        //You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
        final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
        tec.addComparator(new Comparator<RobotAtCriticalSection> () {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                CriticalSection cs = o1.getCriticalSection();
                RobotReport robotReport1 = o1.getRobotReport();
                RobotReport robotReport2 = o2.getRobotReport();
                return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
            }
        });
        tec.addComparator(new Comparator<RobotAtCriticalSection> () {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
            }
        });

        //You probably also want to provide a non-trivial forward model
        //(the default assumes that robots can always stop)
        tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
        tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(2)));


        //Need to setup infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        //Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        //Setup a simple GUI (null means empty map, otherwise provide yaml file)
        String yamlFile = "maps/map1.yaml";
        JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        tec.setVisualization(viz);

        // Footprint
        Coordinate corner1 = new Coordinate(-0.5,0.3);
        Coordinate corner2 = new Coordinate(0.5,0.3);
        Coordinate corner3 = new Coordinate(0.5,-0.3);
        Coordinate corner4 = new Coordinate(-0.5,-0.3);
        tec.setDefaultFootprint(corner1, corner2, corner3, corner4);

        ///Instantiate a simple motion planner
        ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
        //rsp.setMap(yamlFile);
        rsp.setTurningRadius(4.0);
        rsp.setDistanceBetweenPathPoints(0.05);
        rsp.setRadius(0.3);
        rsp.setFootprint(corner1, corner2, corner3, corner4);

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // Problem
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        // Start and Goal Poses
        Pose s1 = new Pose(1.0,1.0,0.5*Math.PI);
        Pose s2 = new Pose(10.0,1.0,0.5*Math.PI);
        Pose m1 = new Pose(7.0,7.0,0.25*Math.PI);
        Pose m2 = new Pose(3.0,7.0,0.25*Math.PI);
        Pose g1 = new Pose(10.0,10.0,0.0);
        Pose g2 = new Pose(1.0,10.0,0.0);

        // Compute paths
        // s1 -> m1
        rsp.setStart(s1);
        rsp.setGoals(m1);
        if (!rsp.plan()) throw new Error ("No path between " + s1 + " and " + m1);
        PoseSteering[] sm1 = rsp.getPath();
        for (PoseSteering ps : sm1) {
            System.out.println(ps.getPose());
        }
        System.out.println("other path");
        // m1 -> g1
        rsp.setStart(m1);
        rsp.setGoals(g1);
        if (!rsp.plan()) throw new Error ("No path between " + m1 + " and " + g1);
        PoseSteering[] mg1 = rsp.getPath();
        System.out.println(mg1);
        for (PoseSteering ps : mg1) {
            System.out.println(ps.getPose());
        }
        // s2 -> m2
        rsp.setStart(s2);
        rsp.setGoals(m2);
        if (!rsp.plan()) throw new Error ("No path between " + s2 + " and " + m2);
        PoseSteering[] sm2 = rsp.getPath();
        // m2 -> g2
        rsp.setStart(m2);
        rsp.setGoals(g2);
        if (!rsp.plan()) throw new Error ("No path between " + m2 + " and " + g2);
        PoseSteering[] mg2 = rsp.getPath();

        // Solve
        tec.placeRobot(1, s1);
        tec.placeRobot(2, s2);

        Missions.enqueueMission(new Mission(1, sm1));
        Missions.enqueueMission(new Mission(2, sm2));

        for (int i = 1; i <= 2; i++) {
            final int robotID = i;
            Thread t = new Thread() {
                @Override
                public void run() {
                    Mission m = Missions.getMission(robotID, 0);
                    synchronized(tec) { tec.addMissions(m); }
                    // Simulate DiMOpt finding other path
                        try { Thread.sleep(5000); }
                        catch (InterruptedException e) { e.printStackTrace(); }
                        // New path computed
                        int lastMissionEndIndex = m.getPath().length-2; //tec.getRobotReport(robotID).getPathIndex();
                        int otherRobotId = (robotID == 1) ? 2 : 1;
                        PoseSteering [] next_path = (robotID == 1) ? mg1 : mg2;
                        PoseSteering [] full_path
                                = ObjectArrays.concat(m.getPath(), next_path, PoseSteering.class);
                        tec.replacePath(robotID, full_path,lastMissionEndIndex,(new HashSet<>(Arrays.asList(otherRobotId))));//
                }
            };
            t.start();
        }







    }


}