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

