package se.oru.coordination.coordination_oru.multirobotplanning;

import com.google.common.collect.ObjectArrays;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.utility.logging.MetaCSPLogging;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.logging.Logger;
import java.util.ArrayList;
public  abstract class AbstractMultirobotPlanning {
    protected class MultirobotProblem{
        public MultirobotProblem(Pose[] start_, Pose[] goal_){
            this.start = start_; this.goal = goal_;
        }
        public Pose[] start = null; // Robot's start configuration
        public Pose[] goal = null;  // Robot's goal configuration
    }
    protected int R; // Number of Robots
    public MultirobotProblem problem;
    protected ArrayList<Coordinate[]> footprints = null; // Robot's footprint corners
    protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
    protected final TrajectoryEnvelopeCoordinatorSimulation tec;
    protected double max_vel, max_acc;
    protected String map_file;
    protected AbstractMultirobotPlanning(
            int R_, double max_vel_, double max_acc_, String map_file_){
        this.R = R_; this.max_vel = max_vel_; this.max_acc = max_acc_; this.map_file = map_file_;
        footprints = new ArrayList<>(R);
        tec = new TrajectoryEnvelopeCoordinatorSimulation(max_vel, max_acc);
    }

    public void addMultirobotProblem(Pose[] start_, Pose[] goal_){
        if(goal_.length != R || start_.length != R){
            metaCSPLogger.warning("The amount of poses should be: " + R);
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
    public abstract boolean plan(MultirobotProblem problem);

    public boolean solve(){
        // Start the multirobot solver, will be outputting multiple files with trajectories

        Thread multirobotSolver = new Thread( ( ) -> {this.plan(this.problem);});
        multirobotSolver.start();

/*        for (int i = 1; i <= R; i++) {
            final int robotID = i;
            Thread t = new Thread() {
                @Override
                public void run() {
                    boolean initialize = true;
                    while(true){
                        Mission mission = null;
                        if(existsNewMission(robotID)){
                            mission = getNewMission(robotID);
                        }
                        if(initialize){ // Add first mission
                            tec.addMissions(mission);
                            initialize = false;
                        } else{ // Concat Mission (receding horizon)
                            tec.addMissions(mission); // replacePath
                        }

                        // Sleep for 1s
                        try { Thread.sleep(1000); }
                        catch (InterruptedException e) { e.printStackTrace(); }
                    }
                }
            };
            t.start();
        }*/


        return true;
    }

    boolean existsNewMission(int robotID){
        // Check if file exists

        return false;
    }

    Mission getNewMission(int robotID){
        PoseSteering[] path = new PoseSteering[0]; // populate this


        return  new Mission(robotID,path);
    }

    /**
     * Get trajectory envelope coordinator and set it up
     * @return
     */
    public TrajectoryEnvelopeCoordinator getTrajectoryEnvelopeCoordinator(){
        return this.tec;
    }

    public void setupTrajectoryEnvelopeCoordinator(){
/*        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // Setup
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        tec.addComparator(new Comparator<RobotAtCriticalSection>() {
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
        for(int r = 1; r < R+1; ++r) {
            tec.setForwardModel(r, new ConstantAccelerationForwardModel(max_acc, max_vel, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
        }

        //Need to setup infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);
        //Start the thread that checks and enforces dependencies at every clock tick
        tec.startInference();

        //Setup a simple GUI (null means empty map, otherwise provide yaml file)
        JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        if(map_file != null) viz.setMap("maps/" +map_file);
        tec.setVisualization(viz);*/

    }
}
