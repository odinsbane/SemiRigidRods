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
    private double[] points = new double[6];
    double width;

    public Spring(Attachment a, Attachment b, double width){
        this.a = a;
        this.b = b;
        setColor(new Color(255, 255, 255, 155));
        this.width = width;
    }


    public Vector getForce(){
        Point ptA = a.getAttachment();
        Point ptB = b.getAttachment();
        Vector v = getWrapped(ptA, ptB);
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
        return getWrapped(a, b).length;
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

    /**
     *
     * @param a
     * @param b
     * @return
     */
    Vector getWrapped(Point a, Point b){
        a.getPosition(points);
        b.getPosition(points, 3);
        double dx = wrap(points[3] - points[0]);
        double dy = wrap(points[4] - points[1]);
        double dz = wrap(points[5] - points[2]);
        return new Vector(dx, dy, dz);
    }

    double wrap(double delta){
        return  delta > width/2 ? delta - width :
                delta < -width/2 ? delta + width : delta;
    }
}