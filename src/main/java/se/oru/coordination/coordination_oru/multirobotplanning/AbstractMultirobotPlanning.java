package se.oru.coordination.coordination_oru.multirobotplanning;

import com.google.common.collect.ObjectArrays;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.utility.logging.MetaCSPLogging;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.*;
import java.util.logging.Logger;

public  abstract class AbstractMultirobotPlanning {
    protected Deque<PoseSteering[]>[] paths;
    protected class MultirobotProblem{
        public MultirobotProblem(Pose[] start_, Pose[] goal_){
            this.start = start_; this.goal = goal_;
        }
        public Pose[] start = null; // Robot's start configuration
        public Pose[] goal = null;  // Robot's goal configuration
    }
    protected int R; // Number of Robots
    protected MultirobotProblem problem;
    protected ArrayList<Coordinate[]> footprints = null; // Robot's footprint corners
    protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
    protected final TrajectoryEnvelopeCoordinatorSimulation tec;
    protected double max_vel, max_acc;
    protected String map_file;
    protected AbstractMultirobotPlanning(
            int R_, double max_vel_, double max_acc_, String map_file_){
        this.R = R_; this.max_vel = max_vel_; this.max_acc = max_acc_; this.map_file = map_file_;
        footprints = new ArrayList<>(R);
        this.paths = (Deque<PoseSteering[]>[]) new Deque[R];
        for(int r = 0; r < R; ++r){
            this.paths[r] = new LinkedList<PoseSteering[]>();
        }
        tec = new TrajectoryEnvelopeCoordinatorSimulation(
                1000,2000,max_vel, max_acc);
    }

    public void addMultirobotProblem(Pose[] start_, Pose[] goal_){
        if(goal_.length != R || start_.length != R){
            metaCSPLogger.warning("The amount of poses should be: " + R);
        }
        this.problem = new MultirobotProblem(start_, goal_);
    }

    /**
     * Robots have been already placed or executed a mission previously
     * @param goal_
     */
    public void addMultirobotProblem(Pose[] goal_){
        if( goal_.length != R ){
            metaCSPLogger.warning("The amount of poses should be: " + R);
        }
        Pose[] start_ = new Pose[R];
        for(int r = 0; r < R; ++r){
            start_[r] = tec.getRobotReport(r).getPose();
        }
        this.problem = new MultirobotProblem(start_, goal_);
    }

    /**
     * Sets all robots footprints to footprint_standard
     * @param footprint_standard
     */
    public void setFootprintEqual(Coordinate[] footprint_standard){
        tec.setDefaultFootprint(footprint_standard);
        for(int r = 0; r < R; ++r) footprints.add( footprint_standard);
    }

    /**
     * This function has to generate a .json file in the form ... , that
     * is going to be read by the coordinator
     * @param problem
     * @return
     */
    protected abstract boolean plan(MultirobotProblem problem);

    public boolean solve(){
        // Start the multirobot solver, will be outputting multiple files with trajectories
        Thread multirobotSolver = new Thread( ( ) -> {
            this.plan(this.problem);
            metaCSPLogger.info("Multi-robot solver finished computing entire multi-robot trajectory");
        });
        multirobotSolver.start();

        synchronized (this) {
            try {
                missionListener();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return true;
    }

    class SimplePath{
        boolean lastPath;
        ArrayList<Double> x,y,z;
        ArrayList<Double> roll, pitch, yaw;
    }
    private boolean missionListener() throws Exception {
        // Gson to read/parse file
        Gson gson = new Gson();
        // Missions Ids
        int[] mission_sequence_id = new int[R];
        Arrays.fill(mission_sequence_id,0);
        // Finished robots
        ArrayList<Boolean> finished = new ArrayList<>(Arrays.asList(new Boolean[R]));
        Collections.fill(finished, false);

        while (true) {
            // Listen for mission path files for each robot
            for (int r = 0; r < R; ++r) {
                if (finished.get(r)) continue; // Robot finished
                String filename = "RH-DiMOpt/bin/t_path_" + r + "_" + mission_sequence_id[r]++;
                File file = new File(filename);
                // Check file
                if (file.exists()) { // If file exists (path was computed)
                    // Convert to simple_path
                    SimplePath path = gson.fromJson(
                            new FileReader(filename),
                            SimplePath.class);
                    //file.delete(); // delete file
                    // Convert to PoseSteering[]
                    int N = path.x.size();
                    PoseSteering[] ps = new PoseSteering[N];
                    for (int k = 0; k < N; ++k) {
                        ps[k] = new PoseSteering(
                                path.x.get(k),
                                path.y.get(k),
                                path.yaw.get(k),
                                0.0);
                    }
                    paths[r].addFirst(ps);
                    finished.set(r, path.lastPath);

                    // handle file
                    file.delete();
                } else{
                    mission_sequence_id[r]--;
                }
            }
            // Test if all robots finished
            if (finished.stream().allMatch(elem -> elem == true)) break;

            // Sleep for 0.2s
            try { Thread.sleep(100); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
        return true;
    }

    boolean existsNewMission(int robotID){
        // Check if file exists
        return !paths[robotID-1].isEmpty();
    }

    Mission getNewMission(int robotID){
        PoseSteering[] path = paths[robotID-1].getLast();
        paths[robotID-1].removeLast();
        return  new Mission(robotID, path);
    }

    /**
     * Get trajectory envelope coordinator and set it up
     * @return
     */
    public TrajectoryEnvelopeCoordinator getTrajectoryEnvelopeCoordinator(){
        return this.tec;
    }

    // Setup the trajectory envelope coordinator tec variable while starts a thread
    // for each robot that addmissions once they are available

    public void setupTrajectoryEnvelopeCoordinator(Pose[] start){
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // Setup
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //tec.setUseInternalCriticalPoints(false);
        //tec.setYieldIfParking(true);
        //tec.setBreakDeadlocks(false, true, true);


        //You probably also want to provide a non-trivial forward model
        //(the default assumes that robots can always stop)
        for(int r = 1; r <= R; ++r) {
            tec.setForwardModel(r, new ConstantAccelerationForwardModel(max_acc, max_vel, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
        }

        //Need to setup infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        //Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        //Setup a simple GUI (null means empty map, otherwise provide yaml file)
        //JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        BrowserVisualization viz = new BrowserVisualization();
        viz.setInitialTransform(40.0, 1.5, 15.0);
        if(map_file != null) viz.setMap("maps/" +map_file);
        tec.setVisualization(viz);




        // Place Robots
        for(int r = 0; r < R; ++r){
            tec.placeRobot(r+1, start[r]);
        }

        // Start robot threads
        for (int i = 1; i <= R; i++) {
            final int robotID = i;
            Thread t = new Thread() {
                @Override
                public void run() {
                    boolean initialize = true;
                    Mission currMission = null;
                    while(true){ // receding horizon loop
                        synchronized (this) {
                            if (existsNewMission(robotID)) {
                                Mission newMission = getNewMission(robotID);
                                if (initialize) { // Add first mission
                                    while(true) { // Finish current mission loop
                                        synchronized (tec) {
                                            if(tec.addMissions(newMission)){
                                                currMission = newMission;
                                                break;
                                            }
                                        }
                                        // Sleep for 0.5s
                                        try { Thread.sleep(500); }
                                        catch (InterruptedException e) { e.printStackTrace(); }
                                    }
                                    initialize = false;
                                } else { // Concat Mission (receding horizon)
         /*                           while(true) {
                                        synchronized (tec) {
                                            if (tec.addMissions(newMission)) {
                                                currMission = newMission;
                                                break;
                                            }
                                        }

                                        // Sleep for 0.5s
                                        try {
                                            Thread.sleep(500);
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }
                                    }*/
                                    PoseSteering[] currPath = currMission.getPath();
                                    PoseSteering[] newPath = newMission.getPath();
                                    // Concat paths
                                    PoseSteering[] concatPath
                                            = new PoseSteering[currPath.length + newPath.length-1];
                                    for(int k = 0; k < currPath.length; ++k) concatPath[k] = currPath[k];
                                    for(int k = 1; k < newPath.length ; ++k) concatPath[currPath.length-1+k] = newPath[k];
                                    Mission concatMission = new Mission(robotID, concatPath);
                                    //HashSet<Integer> otherRobots = new HashSet<>(Arrays.asList(1,2,3));
                                    //otherRobots.remove(robotID);
                                    tec.replacePath(robotID,concatPath,currPath.length-1,false, new HashSet<>() );
                                    currMission = concatMission;
                                }
                            }
                        }
                        // Sleep for 1s
                        try { Thread.sleep(5000); }
                        catch (InterruptedException e) { e.printStackTrace(); }
                    }
                }
            };
            t.start();
        }

    }
}
