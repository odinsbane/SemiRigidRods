package org.orangepalantir;

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
    static double dt = 1e-3;
    static double time = 0;
    static String tag;
    static boolean saving = false;
    static void stepAttachments(Spring s, RigidRodAttachment a, RigidRodAttachment b){
        time += dt;

        double projectedForce = 0;

        Vector v;
        boolean aAttached = Math.abs(a.loc)<=a.rod.length*0.5;
        boolean bAttached = Math.abs(b.loc)<=b.rod.length*0.5;
        if((!aAttached)||(!bAttached)){
            if((!aAttached)&&(!bAttached)){
                return;
            }
            v = Vector.ZERO;
        } else {
            v = s.getForce();
        }

        if(aAttached){
            Vector t0 = a.rod.getTangent(a.loc);
            Vector p = t0.projection(v);

            a.loc += (1 - p.length)*dt;
        }

        if(bAttached) {

            Vector t1 = b.rod.getTangent(b.loc);
            Vector p = t1.projection(v);
            b.loc += (1 - p.length) * dt;
        }

    }

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
        try {

            limit = Double.parseDouble(args[0]);
            stiffnessFactor = Double.parseDouble(args[1]);
            if(args.length>=3) {
                gui=false;
                saving=true;
                out = new File(args[2]);
                if (out.exists()) {
                    if (!out.isDirectory()) {
                        System.out.println("Output is not a directory.");
                        System.out.println("usage: simulation <relax value> directory");
                        System.exit(-1);
                    }
                } else {
                    if (!out.mkdir()) {
                        System.out.println("Cannot create output directory!");
                        System.out.println("usage: simulation <relax value> directory");
                        System.exit(-1);

                    }
                }
            }else{
                gui=true;
                saving=false;
            }
        }catch(Exception e){
            System.out.println("usage: simulation <relax value> directory");
            e.printStackTrace();
            System.exit(-1);
        }

        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 101, 2);
        RigidRod r1 = new RigidRod(new Point(0, 0.3, 0), new Vector(-1, 0, 0),101, 2);
        r0.setBendingStiffness(stiffnessFactor*r0.kappa);
        r1.setBendingStiffness(stiffnessFactor*r1.kappa);
        RigidRodAttachment walkingA = new RigidRodAttachment(-0., r0);
        RigidRodAttachment walkingB = new RigidRodAttachment(0, r1);
        Spring spring0 = new Spring(
                walkingA,
                walkingB
        );

        Spring spring1 = new Spring(
                new RigidRodAttachment(-0.9, r0),
                new RigidRodAttachment(1, r1)
        );

        Spring spring2 = new Spring(
                new RigidRodAttachment(0.9, r0),
                new RigidRodAttachment(-1, r1)
        );

        Spring spring3 = new Spring(
                new RigidRodAttachment(-0.05, r0),
                new RigidRodAttachment(-0.05, r1)
        );

        RodViewer viewer = new RodViewer();

        List<RigidRod> rods = new ArrayList<>();
        List<Spring> springs = new ArrayList<>();

        viewer.addRod(r0);
        viewer.addRod(r1);

        rods.add(r0);
        rods.add(r1);



        viewer.setSelected(r0);

        viewer.addSpring(spring0);
        viewer.addSpring(spring1);
        viewer.addSpring(spring2);
        viewer.addSpring(spring3);

        springs.add(spring0);
        springs.add(spring1);
        springs.add(spring2);
        springs.add(spring3);
        if(gui) {
            EventQueue.invokeLater(viewer::buildGui);
        }
        double lastWrite = -1;
        int count = 0;
        AdaptiveIntegrator integrator = new AdaptiveIntegrator();
        integrator.prepare(rods);
        double s=-1;
        int counter = 0;
        outer:
        while(viewer.displays()){
            for(int j = 0; j<1000; j++){

                s = integrator.step(rods, springs);
                counter++;
                if((s>0 && s<limit)||counter>1e6){
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
                    stepAttachments(spring0, walkingA, walkingB);
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
