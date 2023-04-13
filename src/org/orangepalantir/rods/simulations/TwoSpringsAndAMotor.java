package org.orangepalantir.rods.simulations;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.RodViewer;
import org.orangepalantir.rods.integrators.AdaptiveIntegrator;
import org.orangepalantir.rods.integrators.UpdatableAgent;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;
import org.orangepalantir.rods.interactions.StaticAttachment;
import org.orangepalantir.rods.io.RodIO;

import javax.imageio.ImageIO;
import java.awt.EventQueue;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
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
    final static String usage = "usage: simulation <relax value> <stiffness factor> <links> <points> directory";
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
        int fixedSprings = 2;
        int rodPoints = 5;
        try {

            limit = Double.parseDouble(args[0]);
            stiffnessFactor = Double.parseDouble(args[1]);
            fixedSprings = Integer.parseInt(args[2]);
            rodPoints = Integer.parseInt(args[3]);
            if(args.length>=4) {
                gui=true;
                saving=true;
                out = new File(args[4]);
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
        RigidRod r0 = new RigidRod(new Point(0, 0.4, 0), new Vector(1, 0, 0), rodPoints, length);
        RigidRod r1 = new RigidRod(new Point(0, 0.6, 0), new Vector(-1, 0, 0), rodPoints, length);
        r0.setBendingStiffness(stiffnessFactor*r0.kappa);
        r1.setBendingStiffness(stiffnessFactor*r1.kappa);

        Motor motor = new Motor(0.8, 1000, 0.2, 100, 75, 10);
        motor.bindRod(r0, 0, 0, 100);
        motor.bindRod(r1, 0, 1, 100);

        List<Spring> springs = new ArrayList<>();

        springs.add(motor.springs[0]);
        springs.add(motor.springs[1]);

        RodViewer viewer = new RodViewer();

        List<RigidRod> rods = new ArrayList<>();

        rods.add(r0);
        rods.add(r1);

        double ds = r0.length/(fixedSprings-1);

        double springL = 0.0;
        double[] stuck = new double[3];
        for(int i = 0; i<fixedSprings; i++){
            double loc = -length*0.5 + i*ds*0.5;
            r0.getPoint(loc, stuck);
            Spring s = new Spring(
                    new RigidRodAttachment(loc, r0),
                    new StaticAttachment(new Point(stuck)), 10
            );
            s.setRestLength(springL);
            springs.add(s);
            r1.getPoint(loc, stuck);
            Spring s2 = new Spring(
                    new RigidRodAttachment(loc, r1),
                    new StaticAttachment(new Point(stuck)), 10
            );
            s2.setRestLength(springL);
            springs.add(s2);

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

        List<UpdatableAgent> agents = new ArrayList<>();
        agents.addAll(rods);
        agents.add(motor);

        integrator.prepare(agents);
        double s=-1;
        int counter = 0;
        time = 0;
        outer:
        viewer.repaint();
        while(viewer.displays()){
            for(int j = 0; j<1000; j++){

                s = integrator.step(springs);
                counter++;
                if((s>0 && s<limit)||counter>1e6){
                    time += dt;

                    if(time - lastWrite >= 0.01 ){
                        try(RodIO rio = RodIO.saveRigidRodSimulation();) {
                            String status = String.format("relaxed: %2.2f:  %2.4e     %2.4e , %07d", time, integrator.DT, s, counter);
                            viewer.setStatus(String.format(status));
                            viewer.repaint();
                            if(saving) {
                                String fname = String.format("XX%s-%03d.png", tag, count++);
                                ImageIO.write(viewer.display, "PNG", new File(out, fname));

                                rio.setMotors(List.of(motor));
                                rio.setSprings(springs);
                                rio.setRods(rods);
                                rio.setOutput(Paths.get("dats", String.format("t_%03d.dat", count)));
                                rio.write();
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
                    motor.walk(dt);
                }
            }
            String status = String.format("%2.2f:  %2.4e     %2.4e", time, integrator.DT, s);
            viewer.setStatus(status);
            viewer.repaint();
        }

    }

}
