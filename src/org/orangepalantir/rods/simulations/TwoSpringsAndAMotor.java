package org.orangepalantir.rods.simulations;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.RodViewer;
import org.orangepalantir.rods.integrators.AdaptiveIntegrator;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;

import javax.imageio.ImageIO;
import java.awt.EventQueue;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

/**
 * Created by msmith on 18/08/16.
 */
public class TwoSpringsAndAMotor {
    static double dt = 1e-6;
    static double time = 0;
    static String tag;
    static boolean saving = false;
    final static String usage = "usage: simulation <relax value> <stiffness factor> links directory";
    public static void main(String[] args){
        boolean gui=false;
        tag = new Random().ints().limit(5).mapToObj(i ->{

            int v = i<0?(-i)%26:i%26;
            char a = (char)(v + 'a');
            return a+"";
        }).collect(Collectors.joining());
        double limit = -1;
        File out = null;
        double stiffnessFactor=1;
        int N = 2;
        try {

            limit = Double.parseDouble(args[0]);
            stiffnessFactor = Double.parseDouble(args[1]);
            N = Integer.parseInt(args[2]);
            if(args.length>=4) {
                gui=true;
                saving=true;
                out = new File(args[3]);
                if (out.exists()) {
                    if (!out.isDirectory()) {
                        System.out.println("Output is not a directory.");
                        System.out.println();
                        System.exit(-1);
                    }
                } else {
                    if (!out.mkdir()) {
                        System.out.println(usage);
                        System.exit(-1);

                    }
                }
            }else{
                gui=true;
                saving=false;
            }
        }catch(Exception e){
            System.out.println(usage);
            e.printStackTrace();
            System.exit(-1);
        }
        double length = 2.0;

        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 5, length);
        RigidRod r1 = new RigidRod(new Point(0, 0.2, 0), new Vector(-1, 0, 0), 5, length);
        r0.setBendingStiffness(stiffnessFactor*r0.kappa);
        r1.setBendingStiffness(stiffnessFactor*r1.kappa);

        Motor motor = new Motor(0.8, 1000, 0.2, 100);
        motor.bindRod(r0, 0, 0, 100);
        motor.bindRod(r1, 0, 1, 100);

        List<Spring> springs = new ArrayList<>();

        springs.add(motor.springs[0]);
        springs.add(motor.springs[1]);

        RodViewer viewer = new RodViewer();

        List<RigidRod> rods = new ArrayList<>();

        rods.add(r0);
        rods.add(r1);

        double ds = r0.length/(N-1);

        double springL = 0.2;
        for(int i = 0; i<N; i++){
            Spring s = new Spring(
                    new RigidRodAttachment(-length*0.5 + i*ds, r0),
                    new RigidRodAttachment(length*0.5 - i*ds, r1)
            );
            s.setRestLength(springL);
            springs.add(s);

        }


        for(RigidRod rod: rods){
            viewer.addRod(rod);
        }
        viewer.addRod(motor);

        for(Spring spring: springs){
            viewer.addSpring(spring);
        }

        viewer.setSelected(r0);

        if(gui) {
            EventQueue.invokeLater(viewer::buildGui);
        }
        double lastWrite = -1;
        int count = 0;
        AdaptiveIntegrator integrator = new AdaptiveIntegrator();
        integrator.prepare(rods);
        double s=-1;
        int counter = 0;
        time = 0;
        outer:
        while(viewer.displays()){
            for(int j = 0; j<1000; j++){

                s = integrator.step(rods, springs);
                counter++;
                if((s>0 && s<limit)||counter>1e6){
                    time += dt;
                    if(time - lastWrite >= 0.01 ){
                        try {
                            String status = String.format("relaxed: %2.2f:  %2.4e     %2.4e , %07d", time, integrator.DT, s, counter);
                            viewer.setStatus(String.format(status));
                            viewer.repaint();
                            if(saving) {
                                String fname = String.format("XX%s-%03d.png", tag, count++);
                                ImageIO.write(viewer.display, "PNG", new File(out, fname));
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        lastWrite = time;
                    }
                    //relaxed.
                    if(time>=2) {
                        System.exit(0);
                    }
                    counter = 0;
                }
            }
            String status = String.format("%2.2f:  %2.4e     %2.4e", time, integrator.DT, s);
            viewer.setStatus(status);
            viewer.repaint();
        }

    }

}