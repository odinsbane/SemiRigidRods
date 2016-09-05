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
    protected double s0 = 0.2;
    protected double k = 100;
    protected final Attachment a, b;
    public Spring(Attachment a, Attachment b){
        this.a = a;
        this.b = b;
    }

    protected Vector getForce(){
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

    public void setRestLength(double length) {
        s0 = length;
    }
}