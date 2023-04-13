package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;

/**
 * Created by msmith on 05/09/16.
 */
public class RigidRodAttachment implements Attachment {
    public double loc;
    public RigidRod rod;
    double[] pt = new double[3];
    public RigidRodAttachment(double loc, RigidRod rod){
        this.loc = loc;
        this.rod = rod;
    }
    public void setRod(double loc, RigidRod rod){
        this.rod = rod;
        this.loc = loc;
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

