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

        vForce = new Vector(vForce.length/100 + 0.2, vForce);
    }
    public double getAttachmentLocation(){
        return attachmentPoint;
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

    public RigidRod getRod() {
        return rod;
    }

    public void getActualForce(double[] f){
        f[0] = force[0];
        f[1] = force[1];
        f[2] = force[2];
    }
}

