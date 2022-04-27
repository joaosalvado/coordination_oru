package se.oru.coordination.coordination_oru.multirobotplanning;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.utility.logging.MetaCSPLogging;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;

import java.util.Comparator;
import java.util.logging.Logger;
import java.util.ArrayList;
public  abstract class AbstractMultirobotPlanning {
    protected int R; // Number of Robots
    protected Pose[] start = null; // Robot's start configuration
    protected Pose[] goal = null;  // Robot's goal configuration
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

    public void setMultirobotStart(Pose[] start_){
        if(start_.length != R) metaCSPLogger.warning("The amount of poses should be: " + R);
        start = start_;
    }
    public void setMultirobotGoal(Pose[] goal_){
        if(goal_.length != R) metaCSPLogger.warning("The amount of poses should be: " + R);
        goal = goal_;
    }

    public void setMultirobotProblem(Pose[] start_, Pose[] goal_){
        if(goal_.length != R || start_.length != R){
            metaCSPLogger.warning("The amount of poses should be: " + R);
        }
        goal = goal_;
        start = start_;
    }

    /**
     * Sets all robots footprints to footprint_standard
     * @param footprint_standard
     */
    public void setFootprintEqual(Coordinate[] footprint_standard){
        tec.setDefaultFootprint(footprint_standard);
        for(int r = 0; r < R; ++r) footprints.add( footprint_standard);
    }

    protected boolean isProblemValid(){
        if(start.length != R) return false;
        if( goal.length != R) return false;
        return true;
    }

    public abstract boolean plan();

    public boolean handleSolution(){

        return true;
    }

    /**
     * Get trajectory envelope coordinator and set it up
     * @return
     */
    public TrajectoryEnvelopeCoordinator getTrajectoryEnvelopeCoordinator(){
        return this.tec;
    }

    public void setupTrajectoryEnvelopeCoordinator(){
        ////////////////////////////////////////////////////////////////////////////////////////////////////
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
        if(map_file != null) viz.setMap(map_file);
        tec.setVisualization(viz);

    }
}
