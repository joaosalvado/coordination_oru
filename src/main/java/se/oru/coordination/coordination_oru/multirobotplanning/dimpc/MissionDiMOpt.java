package se.oru.coordination.coordination_oru.multirobotplanning.dimpc;


import com.vividsolutions.jts.algorithm.MinimumBoundingCircle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import org.metacsp.utility.logging.MetaCSPLogging;
import java.util.logging.Logger;
import java.util.ArrayList;


public class MissionDiMOpt {
    protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
    private static final Logger LOGGER = Logger.getLogger( MissionDiMOpt.class.getName() );
    // Helper Classes
    /**
     *  SE2 class to define start and goal robot configurations
     *  x - euclidean coordinate
     *  y - euclidean coordinate
     *  o - orientation (theta)
      */
    public static class SE2 {
        private double x, y, o;
        public SE2(double x_, double y_, double o_) {
            this.x = x_; this.y = y_; this.o = o_;
        }
        public SE2(double ... xyo) {
            if (xyo.length != 3) LOGGER.warning("Provide state length 3!");
            this.x = xyo[0]; this.y = xyo[1]; this.o = xyo[2];
        }
    }

    /**
     *  Polytope is a convex region defined by the intersection of halfspaces
      */
    public static class Polytope {
        private ArrayList<Halfspace> halfspaces;
        /**
         * Halspace is composed of a normal vector (a_x, a_y) of a line crossing y-axis at b
         * a_x * x + a_y * y <= b
         */
        public static class Halfspace {
            private double a_x, a_y, b;
            public Halfspace(double ... vars){
                if(vars.length != 3) LOGGER.warning("Halfspace is defined by 3 vars!");
                this.a_x = vars[0]; this.a_y = vars[1]; this.b = vars[2];
            }
        }
        public Polytope() {
            this.halfspaces = new ArrayList<>();
        }
        public Polytope(Halfspace ... hps){
            this.halfspaces = new ArrayList<>();
            for(Halfspace hp : hps) this.halfspaces.add(hp);
        }
        public void  addHalfSpace(Halfspace hp){ this.halfspaces.add(hp);}
    }

    // SE2 creators
    public SE2 se2(double x, double y, double o) {
        return new SE2(x, y, o);
    }
    public SE2 se2(double... xyo) {
        return new SE2(xyo);
    }

    // Polytope creators
    public Polytope polytope(){
        return new Polytope();
    }
    public Polytope polytope(Polytope.Halfspace ... hps){
        return new Polytope(hps);
    }


    // Mission
    int R; // Number of Robots
    int N; // Discretization variables
    ArrayList<SE2> start; // Starting Configuration
    ArrayList<SE2> goal; // Goal Configuration
    ArrayList<Polytope> freeSpace; // FreeSpace convex region

    public MissionDiMOpt(int R_){
        this.start = new ArrayList<>(R_);
        this.goal =  new ArrayList<>(R_);
        this.freeSpace = new ArrayList<>(R_);
        this.R = R_;
    }
    public MissionDiMOpt setMultirobotStart(SE2 ... states){
        if(states.length != R) metaCSPLogger.warning("Provide " + R + " states");
        for(SE2 state : states) start.add(state);
        return this;
    }
    public MissionDiMOpt setMultirobotGoal(SE2 ... states){
        if(states.length != R) metaCSPLogger.warning("Provide " + R + " states");
        for(SE2 state : states) goal.add(state);
        return this;
    }
    public MissionDiMOpt setFreeSpace( Polytope ... polytopes){
        if(polytopes.length != R) metaCSPLogger.warning("Provide " + R + " polygons");
        for(Polytope pol : polytopes) freeSpace.add(pol);
        return this;
    }

    /**
     * All robots navigate in the same polytope
     * @param polytope
     * @return
     */
    public MissionDiMOpt setSamePolytopeFreeSpace( Polytope polytope){
        for(int r = 0; r < R; ++r) freeSpace.add(polytope);
        return this;
    }
}

