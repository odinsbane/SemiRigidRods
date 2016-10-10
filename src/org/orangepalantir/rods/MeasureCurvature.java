package org.orangepalantir.rods;

import lightgraph.Graph;
import org.orangepalantir.rods.interactions.Spring;
import org.orangepalantir.rods.io.RodIO;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This class will load a file from a .dat file then calculate a curvature distribution histogram.
 *
 * Created by Matt on 05/10/16.
 */
public class MeasureCurvature {
    int BINS = 2500;
    Graph graph;
    Graph graph2;
    static List<double[]> measureCurvatures(List<RigidRod> rods){
        return rods.stream().map(MeasureCurvature::measureCurvature).collect(Collectors.toList());
    }
    static double[] measureCurvature(RigidRod rod){
        double[] curvatures = new double[rod.N-2];
        for(int i = 0; i<rod.N-2; i++){
            curvatures[i] = rod.getCurvature(i+1);
        }
        return curvatures;
    }

    public void measureCurvature(Path path) throws IOException {
        RodIO rodIo = RodIO.loadRigidRod(path);
        List<RigidRod> rods = rodIo.getRods();
        List<double[]> raw = measureCurvatures(rods);

        List<Spring> springs = rodIo.getSprings();

        for(Spring spring: springs){
            spring.applyForces();
        }

        double sum2 = 0;
        for(RigidRod rod: rods){
            sum2 += rod.prepareInternalForces();
        }
        sum2 = sum2/rods.size();

        double min = Double.MAX_VALUE;
        double max = -Double.MIN_VALUE;
        for(double[] r: raw){
            for(double d: r){
                min = d<min?d:min;
                max = d>max?d:max;
            }
        }
        //max=max/50;
        double[] x = new double[BINS];
        double[] y = new double[BINS];
        double[] cum = new double[BINS];
        double dx = (max - min)/BINS;
        for(int i = 0; i<x.length; i++){
            //x[i] = Math.log(dx*(i + 0.5) + min);
            x[i] = dx*(i + 0.5) + min;
        }

        double count = 0;

        for(double[] r: raw){
            for(double d: r){
                int dex = (int)((d - min)/dx);
                if(dex>BINS) continue;

                dex = dex==BINS?dex = BINS-1:dex;
                y[dex]++;
                count++;
            }
        }
        double sum = 0;
        for(int i = 0; i<y.length; i++){
            y[i] = y[i]/count;
            sum += y[i];
            cum[i] = sum;
        }


        if(graph==null){

            graph= new Graph();
            graph.setXLabel("log(curvature (dt/ds))");
            graph.setYLabel("count/total");
            graph.setTitle("Curvature Histogram: " + path);
            graph.setYRange(0, 1);
            graph.addData(x,y).setLabel(String.format("%f",sum2));
            graph.show(true);
        } else{
            graph.addData(x,y).setLabel(String.format("%f",sum2));
            graph.refresh(true);

        }

        if(graph2==null){
            graph2= new Graph();
            graph2.setXLabel("log(curvature (dt/ds))");
            graph2.setYLabel("count/total");
            graph2.setTitle("Cumulative Curvature Distribution: " + path);
            graph2.addData(x,cum).setLabel(String.format("%f",sum2));
            graph2.setYRange(0, 1);
            graph2.show(true);
        } else{
            graph2.addData(x,cum).setLabel(String.format("%f",sum2));
            graph2.refresh(true);
        }
        /*RodViewer viewer = new RodViewer();
        rods.forEach(viewer::addRod);
        viewer.buildGui();
        viewer.repaint();*/
    }

    public static void main(String[] args) throws IOException {
        MeasureCurvature mc = new MeasureCurvature();
        String base;
        base = "1462975597039";  //lf320
        //base = "1462977540713";  //lf500
        //base = "1462986706630"; //lf679
        //base = "1462988565141"; //lf800
        List<Path> paths = Files.list(Paths.get(".")).filter(p->
            p.getFileName().toString().startsWith(base)
        ).collect(Collectors.toList());
        paths.sort((a,b)->Long.compare(a.toFile().lastModified(), b.toFile().lastModified()));
        paths.forEach(p->{
            try {
                mc.measureCurvature(p);
            } catch (IOException e) {
                e.printStackTrace();
            }
        });

    }
}
