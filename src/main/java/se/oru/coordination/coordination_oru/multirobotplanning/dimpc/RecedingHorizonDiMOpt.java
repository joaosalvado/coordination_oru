package se.oru.coordination.coordination_oru.multirobotplanning.dimpc;

import com.vividsolutions.jts.algorithm.MinimumBoundingCircle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import se.oru.coordination.coordination_oru.multirobotplanning.AbstractMultirobotPlanning;

import java.util.ArrayList;
import java.util.Arrays;

public class RecedingHorizonDiMOpt extends AbstractMultirobotPlanning {
    private ArrayList<Double> L; // Circle Robots Diameter;

    public RecedingHorizonDiMOpt(int R){
        super(R);
        L = new ArrayList<>(R);
    }
    @Override
    public boolean plan() {
       if(!isProblemValid()) metaCSPLogger.warning("Problem is not valid!");
       // Populates L
       computeRobotRadius();

       return true;
    }

    private void computeRobotRadius(){
        for(int r = 0; r < R; ++r){
            Coordinate[] coords = this.footprints.get(r);
            GeometryFactory gf = new GeometryFactory();
            Coordinate[] newCoords = new Coordinate[coords.length+1];
            for (int i = 0; i < coords.length; i++) {
                newCoords[i] = coords[i];
            }
            newCoords[newCoords.length-1] = coords[0];
            Polygon pol = gf.createPolygon(coords);
            MinimumBoundingCircle  circleFootprint= new MinimumBoundingCircle(pol);
            L.set(r, circleFootprint.getRadius());
        }
    }

}
