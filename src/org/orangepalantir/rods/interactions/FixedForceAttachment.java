package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.Vector;

/**
 * Created by msmith on 08/09/16.
 */

public class FixedForceAttachment implements Attachment {
    RigidRod rod;
    double attachmentPoint;
    double[] loc = new double[3];
    double[] force;
    Vector vForce;
    public FixedForceAttachment(RigidRod rod, double attachmentPoint, double[] force){
        this.rod = rod;
        this.attachmentPoint = attachmentPoint;
        this.force = force;
        vForce = new Vector(force[0], force[1], force[2]);
        vForce.length = vForce.length/100 + 0.2;
    }
    @Override
    public Point getAttachment() {
        rod.getPoint(attachmentPoint, loc);
        return new Point(loc);
    }
    public Attachment getDanglingEnd(){
        return ()->this.getAttachment().add(vForce);
    }
    @Override
    public void applyForce(Vector v) {
        rod.applyForce(force[0], force[1], force[2], attachmentPoint);
    }
}

