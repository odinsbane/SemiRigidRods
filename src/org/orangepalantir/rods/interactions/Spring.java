package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.Point;

import java.awt.Color;

/**
 * Created by msmith on 17/08/16.
 */
public class Spring {
    protected double s0 = 0.2;
    protected double k = 100;
    public final Attachment a, b;
    private Color color;

    public Spring(Attachment a, Attachment b){
        this.a = a;
        this.b = b;
        setColor(new Color(255, 255, 255, 155));
    }



    public Vector getForce(){
        Vector v = new Vector(a.getAttachment(), b.getAttachment());
        double factor = v.length - s0;
        v.length = factor*k;
        return v;
    }

    public void applyForces(){
        Vector force = getForce();
        a.applyForce(force);
        force.length = -force.length;
        b.applyForce(force);
    }
    double separation(Point a, Point b){
        return Math.sqrt((a.x - b.x)*(a.x-b.x) + (a.y - b.y)*(a.y-b.y) + (a.z - b.z)*(a.z - b.z));
    }
    public double getEnergy(){
        double s = separation(a.getAttachment(), b.getAttachment()) - s0;
        return 0.5*k* s*s;
    }

    public void setRestLength(double length) {
        s0 = length;
    }

    public void setStiffness(double stiffness){
        k = stiffness;
    }

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public double getRestLength() {
        return s0;
    }

    public double getStiffness() {
        return k;
    }
}