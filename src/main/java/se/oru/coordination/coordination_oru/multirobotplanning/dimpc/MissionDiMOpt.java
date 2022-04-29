package se.oru.coordination.coordination_oru.multirobotplanning.dimpc;


import com.vividsolutions.jts.algorithm.MinimumBoundingCircle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import org.metacsp.utility.logging.MetaCSPLogging;
import java.util.logging.Logger;
import java.util.ArrayList;


public class MissionDiMOpt {
    private static final Logger LOGGER = Logger.getLogger( MissionDiMOpt.class.getName() );

    // Mission
    private int R; // Number of Robots
    private int N; // Discretization
    private double v_max;
    private double v_std;
    private ArrayList<ArrayList<Double>> start; // Starting Configuration
    private ArrayList<ArrayList<Double>> goal; // Goal Configuration
    private String mapFile;
    //ArrayList<Polytope> freeSpace; // FreeSpace convex region
    private ArrayList<Double> L; // Robot's radius


    public MissionDiMOpt(int R_){
        this.start = new ArrayList<>(R_);
        this.goal =  new ArrayList<>(R_);
        //this.freeSpace = new ArrayList<>(R_);
        this.R = R_;
    }
    public MissionDiMOpt setParams(int N_, double v_std_, double v_max_){
        this.N = N_; this.v_std = v_std_; this.v_max = v_max_;
        return this;
    }

    public MissionDiMOpt setMultirobotRadius(ArrayList<Double> Ls){
        this.L = Ls;
        return this;
    }
    public MissionDiMOpt setMultirobotStart( ArrayList<ArrayList<Double>> states){
        if(states.size() != R) LOGGER.warning("Provide " + R + " states");
        start = states;
        return this;
    }
    public MissionDiMOpt setMultirobotGoal( ArrayList<ArrayList<Double>> states ){
        if(states.size() != R) LOGGER.warning("Provide " + R + " states");
        goal = states;
        return this;
    }
    public MissionDiMOpt setMap(String filename){
        this.mapFile = filename;
        return this;
    }

}

