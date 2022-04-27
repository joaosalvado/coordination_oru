package se.oru.coordination.coordination_oru.multirobotplanning.dimpc;


import org.metacsp.utility.logging.MetaCSPLogging;
import java.util.logging.Logger;
import java.util.ArrayList;


public class MissionDiMOpt {
    protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
    // Helper Classes
    /**
     *  SE2 class to define start and goal robot configurations
     *  x - euclidean coordinate
     *  y - euclidean coordinate
     *  o - orientation (theta)
      */
    private class SE2 {
        public SE2(double x_, double y_, double o_) {
            this.x = x_;
            this.y = y_;
            this.o = o_;
        }

        public SE2(double... xyo) {
            if (xyo.length != 3) metaCSPLogger.warning("Provide state length 3!");
            this.x = xyo[0];
            this.y = xyo[1];
            this.o = xyo[2];
        }

        private double x;
        private double y;
        private double o;
    }

    /**
     *  Polytope is a convex region defined by the intersection of halfspaces
      */
    private class Polytope {
        /**
         * Halspace is composed of a normal vector (a_x, a_y) of a line crossing y-axis at b
         * a_x * x + a_y * y <= b
         */
        public class Halfspace {
            Halfspace(double ... vars){
                if(vars.length != 3) metaCSPLogger.warning("Halfspace is defined by 3 vars!");
                this.a_x = vars[0]; this.a_y = vars[1]; this.b = vars[2];
            }
            double a_x;
            double a_y;
            double b;
        }
        public Polytope() {
            this.halfspaces = new ArrayList<>();
        }
        public Polytope(Halfspace ... hps){
            this.halfspaces = new ArrayList<>();
            for(Halfspace hp : hps){
                this.halfspaces.add(hp);
            }
        }
        public void  addHalfSpace(Halfspace hp){
           this.halfspaces.add(hp);
        }

        ArrayList<Halfspace> halfspaces;
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
    int N; // Discretization variables
    ArrayList<SE2> start; // Starting Configuration
    ArrayList<SE2> goal; // Goal Configuration
    ArrayList<Polytope> freeSpace; // FreeSpace convex region

    public MissionDiMOpt(int R){
        start = new ArrayList<>(R);
        goal =  new ArrayList<>(R);
        freeSpace = new ArrayList<>(R);
    }

    //MissionDiMOpt setStart()



}

