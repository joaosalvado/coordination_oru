package se.oru.coordination.coordination_oru.tests.clean;

import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "One-shot navigation of 3 robots with different footprints coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class ThreeDifferentRobotsLinear {

	public static void main(String[] args) throws InterruptedException {

		final HashMap<Integer,Integer> robotIDtoPriority = new HashMap<Integer, Integer>();
		
		//biggest first
		robotIDtoPriority.put(1, 2); //Med
		robotIDtoPriority.put(2, 3); //Small
		robotIDtoPriority.put(3, 1); //Big
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

		//CLOSEST FIRST
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		
		//IDs
//		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
//			@Override
//			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
//				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
//			}
//		});
		
		//FIXED PRIO
//		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
//			@Override
//			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
//				return robotIDtoPriority.get(o1.getRobotReport().getRobotID()) - robotIDtoPriority.get(o2.getRobotReport().getRobotID());
//			}
//		});
		
		//FCFS
//		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
//			@Override
//			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
//				return (int)Math.signum(o1.getRobotReport().getDistanceTraveled() - o1.getRobotReport().getDistanceTraveled());
//			}
//		});
		
		
		tec.setYieldIfParking(true);
		
//		NetworkConfiguration.setDelays(0,3000);
//		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.1;
//		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		//You can set a footprint that is specific for each robot
		Coordinate[] fp1 = new Coordinate[] {
				new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5)
		};
		Coordinate[] fp2 = new Coordinate[] {
				new Coordinate(0.36, 0.0),
				new Coordinate(0.18, 0.36),
				new Coordinate(-0.18, 0.36),
				new Coordinate(-0.36, 0.0),
				new Coordinate(-0.18, -0.36),
				new Coordinate(0.18, -0.36)
		};
		Coordinate[] fp3 = new Coordinate[] {
				new Coordinate(-2.0,0.9),
				new Coordinate(2.0,0.9),
				new Coordinate(2.0,-0.9),
				new Coordinate(-2.0,-0.9)
		};
		tec.setFootprint(1,fp1);
		tec.setFootprint(2,fp2);
		tec.setFootprint(3,fp3);

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(2)));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(3)));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(49, 5, 0);
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(true);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);

		Pose startPoseRobot1 = new Pose(4.0,6.0,0.0);
		Pose goalPoseRobot1 = new Pose(16.0,15.0,Math.PI/4);
		Pose startPoseRobot2 = new Pose(6.0,16.0,-Math.PI/4);
		Pose goalPoseRobot2 = new Pose(25.0,3.0,-Math.PI/4);
		Pose startPoseRobot3 =new Pose(9.0,6.0,Math.PI/2);
		Pose goalPoseRobot3 =  new Pose(21.0,3.0,-Math.PI/2);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, goalPoseRobot3);

		rsp.setFootprint(fp1);
		rsp.setStart(startPoseRobot1);
		rsp.setGoals(goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		Missions.enqueueMission(new Mission(1,rsp.getPath()));
		Missions.enqueueMission(new Mission(1,rsp.getPathInv()));

		rsp.setFootprint(fp2);
		rsp.setStart(startPoseRobot2);
		rsp.setGoals(goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		Missions.enqueueMission(new Mission(2,rsp.getPath()));
		Missions.enqueueMission(new Mission(2,rsp.getPathInv()));

		rsp.setFootprint(fp3);
		rsp.setStart(startPoseRobot3);
		rsp.setGoals(goalPoseRobot3);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
		Missions.enqueueMission(new Mission(3,rsp.getPathInv()));
		Missions.enqueueMission(new Mission(3,rsp.getPath()));
		
		
		tec.setBreakDeadlocks(true, false, false);
		tec.setMotionPlanner(1, rsp);
		tec.setMotionPlanner(2, rsp);
		tec.setMotionPlanner(3, rsp);

		Thread.sleep(6000);
		
		Missions.startMissionDispatchers(tec, 1,2,3);

	}

}