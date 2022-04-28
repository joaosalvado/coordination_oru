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
import java.util.Optional;

import com.google.gson.Gson;

public class RecedingHorizonDiMOpt extends AbstractMultirobotPlanning {
    private ArrayList<Double> L; // Circle Robots Diameter;

    public RecedingHorizonDiMOpt(int R, double max_vel, double max_acc, String map){
        super(R, max_vel, max_acc, map);
        L = new ArrayList<>(R);
    }
    @Override
    public boolean plan(MultirobotProblem problem) {
       // Populates L
       computeRobotRadius();
       writeProblemFile(problem);
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


    void writeProblemFile(MultirobotProblem problem){
        // Create DiMopt Mission
        MissionDiMOpt missionDiMOpt = new MissionDiMOpt(R);
        ArrayList<MissionDiMOpt.SE2> start_dimopt = new ArrayList<>();
        ArrayList<MissionDiMOpt.SE2> goal_dimopt = new ArrayList<>();
        for(int r = 0; r < R; ++r){
            start_dimopt.add(
                    new MissionDiMOpt.SE2(
                            problem.start[r].getX(),
                            problem.start[r].getY(),
                            problem.start[r].getYaw()) );
            goal_dimopt.add(
                    new MissionDiMOpt.SE2(
                            problem.goal[r].getX(),
                            problem.goal[r].getY(),
                            problem.start[r].getYaw()) );
        }
        missionDiMOpt
                .setMultirobotStart(start_dimopt.toArray(new MissionDiMOpt.SE2[0]))
                .setMultirobotGoal(goal_dimopt.toArray(new MissionDiMOpt.SE2[0]))
                .setMap(this.map_file);
        // write gson file
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(missionDiMOpt);

        try (PrintWriter out = new PrintWriter("RH-DiMOpt/dimopt_mission.txt")) {
            out.println(json);
        } catch (Exception e){ };

    }

    void startDiMOpt(){
        ProcessBuilder p = new ProcessBuilder();
        /*p.command("/bin/bash", "-l", "-c", "mpirun -np 6 --use-hwthread-cpus --oversubscribe ./distributed_scp circle_6 0.5pol.png");
        try{
            p.start();
        }
        catch(Exception e){}
*/

        runCommand("ls -la");
        runCommand( "" );
    }

    private void runCommand(String... command) {
        ProcessBuilder processBuilder
                = new ProcessBuilder().command(
                        "sh", "-c",
                "mpirun -np 6 --use-hwthread-cpus --oversubscribe ./RH-DiMOpt/distributed_scp circle_6 0.5pol.png");
        try {
            Process process = processBuilder.start();

            //read the output
            InputStreamReader inputStreamReader = new InputStreamReader(process.getInputStream());
            BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
            String output = null;
            while ((output = bufferedReader.readLine()) != null) {
                System.out.println(output);
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

}
