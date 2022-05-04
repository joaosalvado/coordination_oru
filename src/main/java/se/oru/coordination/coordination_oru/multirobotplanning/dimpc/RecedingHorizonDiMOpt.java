package se.oru.coordination.coordination_oru.multirobotplanning.dimpc;

import com.google.gson.GsonBuilder;
import com.vividsolutions.jts.algorithm.MinimumBoundingCircle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import org.jgrapht.util.MathUtil;
import se.oru.coordination.coordination_oru.multirobotplanning.AbstractMultirobotPlanning;
import se.oru.coordination.coordination_oru.multirobotplanning.dimpc.MissionDiMOpt;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;

import com.google.gson.Gson;

public class RecedingHorizonDiMOpt extends AbstractMultirobotPlanning {
    private ArrayList<Double> L; // Circle Robots Diameter;
    private int N = 20;
    public RecedingHorizonDiMOpt(int R, double max_vel, double max_acc, String map){
        super(R, max_vel, max_acc, map);
        L = new ArrayList<>(R);
    }

    @Override
    protected boolean plan(MultirobotProblem problem) {
        for(int r = 0; r < R; ++r){
            tec.placeRobot(r+1,problem.start[r]);
        }
       // Populates L
       computeRobotRadius();
       // Writes a .json file with problem
       writeProblemFile(problem);
       // Start dimopt executable that reads .json file and solves the problem
       startDiMOpt();
       return true;
    }

    private void computeRobotRadius(){
        if(L.size() == R) return; // already computed
        for(int r = 0; r < R; ++r){
            Coordinate[] coords = this.footprints.get(r);
            GeometryFactory gf = new GeometryFactory();
            Coordinate[] newCoords = new Coordinate[coords.length+1];
            for (int i = 0; i < coords.length; i++) newCoords[i] = coords[i];
            newCoords[newCoords.length-1] = coords[0];
            Polygon pol = gf.createPolygon(newCoords);
            MinimumBoundingCircle  circleFootprint= new MinimumBoundingCircle(pol);
            L.add( circleFootprint.getRadius());
        }
    }


    private void writeProblemFile(MultirobotProblem problem){
        // Create DiMopt Mission
        MissionDiMOpt missionDiMOpt = new MissionDiMOpt(R);
        ArrayList<ArrayList<Double>> start_dimopt = new ArrayList<>();
        ArrayList<ArrayList<Double>> goal_dimopt = new ArrayList<>();
        for(int r = 0; r < R; ++r){
            start_dimopt.add(
                    new ArrayList<>(Arrays.asList(
                            problem.start[r].getX(),
                            problem.start[r].getY(),
                            problem.start[r].getYaw()) ) );
            goal_dimopt.add(
                    new ArrayList<>(Arrays.asList(
                            problem.goal[r].getX(),
                            problem.goal[r].getY(),
                            problem.start[r].getYaw()) ) );
        }
        missionDiMOpt
                .setMultirobotStart(start_dimopt)
                .setMultirobotGoal(goal_dimopt)
                .setParams(N,max_vel/2, max_vel)
                .setMultirobotRadius(L)
                .setMap(this.map_file);
        // write gson file
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(missionDiMOpt);

        try (PrintWriter out = new PrintWriter("RH-DiMOpt/bin/dimopt_mission.txt")) {
            out.println(json);
        } catch (Exception e){ };

    }

    private void startDiMOpt(){
        ProcessBuilder p = new ProcessBuilder();

        String cmd = "mpirun -np " + this.R + " --use-hwthread-cpus --oversubscribe ./RH-DiMOpt/bin/rhdimopt_coordoru";
        runCommand("/bin/bash", "-l", "-c", cmd);

    }

    private void runCommand(String... cmd) {
        ProcessBuilder processBuilder
                = new ProcessBuilder().command( cmd );
        try {
            Process process = processBuilder.start();

            //read the output
            InputStreamReader inputStreamReader = new InputStreamReader(process.getInputStream());
            BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
            String output = null;
            while ((output = bufferedReader.readLine()) != null) {
                //System.out.println(output); //Uncomment to see Dimopt
            }

            //wait for the process to complete
            process.waitFor();

            //close the resources
            bufferedReader.close();
            process.destroy();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Set the amount of disctretizaiton states.
     * The higher the longer is the receding horizon at the cost of computational complexity increasing!
     * @param N_
     */
    public void setN(int N_){ this.N = N_;}

}
