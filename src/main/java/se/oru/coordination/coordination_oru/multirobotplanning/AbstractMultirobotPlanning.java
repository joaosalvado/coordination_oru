package se.oru.coordination.coordination_oru.multirobotplanning;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import org.metacsp.utility.logging.MetaCSPLogging;
import java.util.logging.Logger;
import java.util.ArrayList;
public  abstract class AbstractMultirobotPlanning {
    protected int R; // Number of Robots
    protected Pose[] start = null; // Robot's start configuration
    protected Pose[] goal = null;  // Robot's goal configuration
    protected ArrayList<Coordinate[]> footprints = null; // Robot's footprint corners
    protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());

    protected AbstractMultirobotPlanning(int R_){
        this.R = R_;
        footprints = new ArrayList<>(R);
    }

    public void setMultirobotStart(Pose[] start_){
        if(start_.length != R) {
            metaCSPLogger.warning("The amount of poses should be: " + R);
        }
        start = start_;
    }
    public void setMultirobotGoal(Pose[] goal_){
        if(goal_.length != R) {
            metaCSPLogger.warning("The amount of poses should be: " + R);
        }
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
        for(int r = 0; r < R; ++r){
            footprints.set(r, footprint_standard);
        }
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
}
