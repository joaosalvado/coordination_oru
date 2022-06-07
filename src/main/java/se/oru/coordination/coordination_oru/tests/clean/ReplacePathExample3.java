package se.oru.coordination.coordination_oru.tests.clean;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example of replacing a path midway.")
public class ReplacePathExample3 {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 1.0;
		
		//Instantiate a trajectory envelope coordinator.
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, MAX_VEL,MAX_ACCEL);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(true);
		tec.setBreakDeadlocks(false, true, true);
		//MetaCSPLogging.setLevel(TrajectoryEnvelopeCoordinator.class, Level.FINEST);

		// Footprint
		double xl = 0.5;
		double yl = 0.3;
		Coordinate corner1 = new Coordinate(-xl,yl);
		Coordinate corner2 = new Coordinate(xl,yl);
		Coordinate corner3 = new Coordinate(xl,-yl);
		Coordinate corner4 = new Coordinate(-xl,-yl);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String map_file = "map-empty.yaml";
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(40.0, 1.5, 15.0);
		viz.setMap("maps/" +map_file);
		tec.setVisualization(viz);


		// Locations
		Pose s1 = new Pose(3.0,7.0,0.0);	// start robot1
		Pose s2 = new Pose( 1.0,8.0,-0.5*Math.PI); // start robot2
		Pose m1 =  new Pose(2.0,4.0,0.5*Math.PI); // middle robot1
		Pose m2 =  new Pose(4.5,4.0,0.25*Math.PI); // middle robot2
		Pose g1 = new Pose( 1.0, 1.0,Math.PI); // goal robot 1
		Pose g2 = new Pose(8.0, 1.0,0.5 *Math.PI); // goal robot 2


		// 1 -  Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		//rsp.setMap(yamlFile);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.05);
		rsp.setRadius(0.1);
		rsp.setFootprint(corner1, corner2, corner3, corner4);


		// 2 - Compute Simple One-step Paths
		// s1 -> m1
		rsp.setStart(s1);
		rsp.setGoals(m1);
		if (!rsp.plan()) throw new Error ("No path between " + s1 + " and " + m1);
		PoseSteering[] sm1 = rsp.getPath();
		// m1 -> g1
		rsp.setStart(m1);
		rsp.setGoals(g1);
		if (!rsp.plan()) throw new Error ("No path between " + s1 + " and " + m1);
		PoseSteering[] mg1 = rsp.getPath();
		// s2 -> m2
		rsp.setStart(s2);
		rsp.setGoals(m2);
		if (!rsp.plan()) throw new Error ("No path between " + s1 + " and " + m1);
		PoseSteering[] sm2 = rsp.getPath();
		// m2 -> g2
		rsp.setStart(m2);
		rsp.setGoals(g2);
		if (!rsp.plan()) throw new Error ("No path between " + s1 + " and " + m1);
		PoseSteering[] mg2 = rsp.getPath();

		// 3 - Concat paths
		// Robot 1
		PoseSteering[] sg1 = new PoseSteering[sm1.length + mg1.length -1];
		for (int i = 0; i < sm1.length; i++) sg1[i] = sm1[i];
		for (int i = 1; i < mg1.length; i++) sg1[sm1.length-1+i] = mg1[i];
		// Robot 2
		PoseSteering[] sg2 = new PoseSteering[sm2.length + mg2.length -1];
		for (int i = 0; i < sm2.length; i++) sg2[i] = sm2[i];
		for (int i = 1; i < mg2.length; i++) sg2[sm2.length-1+i] = mg2[i];
		// Initial Missions
		Mission mission_sm1 = new Mission(1, sm1);
		Mission mission_sm2 = new Mission(2, sm2);

		// These are the full missions
		Mission mission_sg1 = new Mission(1, sg1);
		Mission mission_sg2 = new Mission(2, sg2);

		// 4 - Setup tec and place robots
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.placeRobot(1, s1);
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.placeRobot(2, s2);
		tec.setDefaultFootprint(corner1,corner2,corner3,corner4);

		// 5 - Add Initial missions
		tec.addMissions(mission_sm1,mission_sm2);

		Thread.sleep(7000);

		// 6 - Replace path
		tec.replacePath(1,sg1, sm1.length-1,false,null);
		tec.replacePath(2,sg2, sm2.length-1,false,null);

	}

}
