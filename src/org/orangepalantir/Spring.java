package org.orangepalantir;

import javax.imageio.ImageIO;
import java.awt.EventQueue;
import java.io.File;
import java.io.IOException;
import java.util.Random;
import java.util.stream.Collectors;

/**
 * Created by msmith on 17/08/16.
 */
public class Spring {
    double s0 = 0.2;
    double k = 100;
    final Attachment a, b;
    static double dt = 1e-3;
    static double time = 0;
    static String tag;
    public Spring(Attachment a, Attachment b){
        this.a = a;
        this.b = b;
    }

    Vector getForce(){
        Vector v = new Vector(a.getAttachment(), b.getAttachment());
        double factor = v.length - s0;
        v.length = factor*k;
        return v;
    }

    void applyForces(){
        Vector force = getForce();
        a.applyForce(force);
        force.length = -force.length;
        b.applyForce(force);
    }

    public static void main(String[] args){
        tag = new Random().ints().limit(5).mapToObj(i ->{

            int v = i<0?(-i)%26:i%26;
            char a = (char)(v + 'a');
            return a+"";
        }).collect(Collectors.joining());
        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 51, 2);
        RigidRod r1 = new RigidRod(new Point(0, 0.3, 0), new Vector(-1, 0, 0), 51, 2);

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


        RodViewer viewer = new RodViewer();

        viewer.addRod(r0);
        viewer.addRod(r1);
        viewer.setSelected(r0);

        viewer.addSpring(spring0);
        viewer.addSpring(spring1);
        viewer.addSpring(spring2);


        EventQueue.invokeLater(()->{
            viewer.buildGui();
        });
        double lastWrite = -1;
        int count = 0;
        outer:
        while(viewer.displays()){
            double s = 0;
            for(int j = 0; j<1000; j++){
                spring0.applyForces();
                spring1.applyForces();
                spring2.applyForces();
                s = r0.relax();
                s += r1.relax();

                r0.clearForces();
                r1.clearForces();

                if(s<1e-2){
                        if(time - lastWrite >= 0.01 ){
                            try {
                                viewer.setStatus(String.format("%2.2f: %e", time, s));
                                viewer.repaint();
                                String fname = String.format("XX%s-%03d.png", tag, count++);
                                ImageIO.write(viewer.display, "PNG", new File(fname));
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
                }
            }
            viewer.setStatus(time + ":  " + s);
            viewer.repaint();
        }

    }

    static void stepAttachments(Spring s, RigidRodAttachment a, RigidRodAttachment b){
        time += dt;
        Vector v = s.getForce();
        Vector t0 = a.rod.getTangent(a.loc);
        Vector p = t0.projection(v);
        a.loc += (1 - p.length)*dt;

        Vector t1 = b.rod.getTangent(b.loc);
        p = t1.projection(v);
        b.loc += (1 - p.length)*dt;

    }
}

interface Attachment{
    Point getAttachment();
    void applyForce(Vector v);
}

class RigidRodAttachment implements Attachment{
    double loc;
    RigidRod rod;
    double[] pt = new double[3];
    public RigidRodAttachment(double loc, RigidRod rod){
        this.loc = loc;
        this.rod = rod;
    }
    @Override
    public Point getAttachment() {

        rod.getPoint(loc, pt);
        return new Point(pt);
    }

    @Override
    public void applyForce(Vector v) {
        rod.applyForce(v.dx*v.length, v.dy*v.length, v.dz*v.length, loc);
    }
}

