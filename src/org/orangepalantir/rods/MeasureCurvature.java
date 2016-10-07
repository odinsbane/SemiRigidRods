package org.orangepalantir.rods;

import lightgraph.Graph;
import org.orangepalantir.rods.io.RodIO;

import java.io.IOException;
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

    public void measureCurvature(String path) throws IOException {
        RodIO rodIo = RodIO.loadRigidRod(Paths.get(path));
        List<RigidRod> rods = rodIo.getRods();

        List<double[]> raw = measureCurvatures(rods);
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
            x[i] = Math.log(dx*(i + 0.5) + min);
            //x[i] = dx*(i + 0.5) + min;
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


        Graph graph = new Graph();
        graph.setXLabel("log(curvature (dt/ds))");
        graph.setYLabel("count/total");
        graph.setTitle("Curvature Histogram: " + path);
        graph.addData(x,y);
        graph.show(true);

        Graph graph2 = new Graph();
        graph2.setXLabel("log(curvature (dt/ds))");
        graph2.setYLabel("count/total");
        graph2.setTitle("Cumulative Curvature Distribution: " + path);
        graph2.addData(x,cum);
        graph2.show(true);

        RodViewer viewer = new RodViewer();
        rods.forEach(viewer::addRod);
        viewer.buildGui();
        viewer.repaint();
    }

    public static void main(String[] args) throws IOException {
        MeasureCurvature mc = new MeasureCurvature();
        mc.measureCurvature("14629755970391000001.dat");
        mc.measureCurvature("14629775407131000001.dat");
        mc.measureCurvature("14629867066301000001.dat");
        mc.measureCurvature("14629885651411000001.dat");
        mc.measureCurvature("1462976985411600000.dat");
        mc.measureCurvature("14629769854111000001.dat");



    }
}