package se.oru.coordination.coordination_oru.tests.clean;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example of replacing a path midway.")
public class ReplacePathExample {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 4.0;
		
		//Instantiate a trajectory envelope coordinator.
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, MAX_VEL,MAX_ACCEL);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(true);
		tec.setBreakDeadlocks(false, true, true);
		//MetaCSPLogging.setLevel(TrajectoryEnvelopeCoordinator.class, Level.FINEST);
		
		//Setup the network parameters
//		NetworkConfiguration.setDelays(10, 500);
//		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
//		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 1e-2);

		double xl = 1.0;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = null;
		yamlFile = "maps/map-partial-2.yaml";	
		
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(yamlFile);
		viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);
		
		Missions.loadRoadMap("missions/icaps_locations_and_paths_4.txt");

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);
	
		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);
		
		
		//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
		tec.setMotionPlanner(1, rsp);
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.placeRobot(1, Missions.getLocationPose("L_0"));
		
		PoseSteering[] initialPath = null;		
		rsp.setStart(Missions.getLocationPose("L_0"));
		rsp.setGoals(Missions.getLocationPose("R_0"));
		rsp.plan();
		if (rsp.getPath() == null) throw new Error("No path found.");
		initialPath = rsp.getPath();
	
		int replacementIndex = initialPath.length*2/3;
		Pose replacementPose = initialPath[replacementIndex].getPose();
		rsp.setStart(replacementPose);
		rsp.setGoals(Missions.getLocationPose("R_4"));
		rsp.plan();
		if (rsp.getPath() == null) throw new Error("No path found.");
		PoseSteering[] replacementPath = new PoseSteering[replacementIndex+rsp.getPath().length];
		for (int i = 0; i < replacementIndex; i++) replacementPath[i] = initialPath[i];
		for (int i = 0; i < rsp.getPath().length; i++) replacementPath[i+replacementIndex] = rsp.getPath()[i];

		Mission m = new Mission(1, initialPath);
		Missions.enqueueMission(m);

		Missions.startMissionDispatchers(tec, false, 1);

		int buffer = 20;
		while (tec.getRobotReport(1).getPathIndex() < replacementIndex-buffer) Thread.sleep(50);
		
		tec.replacePath(1,replacementPath,replacementIndex,false,null);

	}

}
