package org.orangepalantir;

import lightgraph.Graph;

import javax.imageio.ImageIO;
import java.awt.EventQueue;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

/**
 * Created by msmith on 26/09/16.
 */
public class TwoRodsAndASpring {

    public static void main(String[] args){
        boolean gui=true;
        double limit = -1;
        int N = 2;

        double length = 2.0;

        List<RigidRod> rods = new ArrayList<>();

        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 11, length);
        RigidRod r1 = new RigidRod(new Point(0, 0.3, 0), new Vector(-1, 0, 0), 11, length);

        rods.add(r0);
        rods.add(r1);

        List<Spring> springs = new ArrayList<>();

        Spring s0 = new Spring(
            new RigidRodAttachment(0, r0),
            new RigidRodAttachment(0, r1)
        );

        s0.setRestLength(0);
        Spring s1 = new Spring(
            new StaticAttachment(new Point(-1, 0, 0)),
            new RigidRodAttachment(-1, r0)
        );
        s1.setRestLength(0);

        Spring s2 = new Spring(
                new StaticAttachment(new Point(1, 0.0, 0)),
                new RigidRodAttachment(1, r0)
        );
        s2.setRestLength(0);

        Spring s3 = new Spring(
                new StaticAttachment(new Point(1, 0.5, 0)),
                new RigidRodAttachment(-1, r1)
        );
        s3.setRestLength(0);

        Spring s4 = new Spring(
                new StaticAttachment(new Point(-1, 0.5, 0)),
                new RigidRodAttachment(1, r1)
        );
        s4.setRestLength(0);

        RodViewer viewer = new RodViewer();



        springs.add(s0);
        springs.add(s1);
        springs.add(s2);
        springs.add(s3);
        springs.add(s4);


        for(RigidRod rod: rods){
            viewer.addRod(rod);
        }

        for(Spring spring: springs){
            viewer.addSpring(spring);
        }

        viewer.setSelected(r0);

        if(gui) {
            EventQueue.invokeLater(viewer::buildGui);
        }
        AdaptiveIntegrator integrator = new AdaptiveIntegrator();
        integrator.prepare(rods);
        double s=-1;
        int counter = 0;
        Graph g = new Graph();
        double[] xs = new double[20];
        for(int i = 0; i<xs.length; i++){
            xs[i] = i;
        }
        double[] ys = new double[20];
        double[] y2s = new double[20];
        g.addData(xs, ys);
        g.addData(xs, y2s);
        //g.setYRange(0, 0.5);
        g.show(false);
        int dex = 0;
        outer:
        while(viewer.displays()){
            for(int j = 0; j<1000; j++){

                s = integrator.step(rods, springs);
                counter++;


                if((s>0 && s<limit)||counter>1e9){
                    break outer;
                }
            }

            double es = 0;
            for(Spring spring: springs){
                es += spring.getEnergy();
            }

            double rb = 0;
            for(RigidRod rod: rods){
                rb += rod.calculateBendEnergy();
            }

            double rs = 0;
            for(RigidRod rod : rods){
                rs += rod.calculateStretchEnergy();
            }


            dex = ++dex%xs.length;
            ys[dex] = es;
            y2s[dex] = rb + rs;

            g.getDataSet(0).setData(xs, ys);
            g.getDataSet(1).setData(xs, y2s);
            g.refresh(true);

            String status = String.format("dt: %2.4e oeq: %2.4e, spring: %2.4e rs: %2.4e rb: %2.4e", integrator.DT, s, es, rb, rs);
            viewer.setStatus(status);
            viewer.repaint();
        }

    }

}
