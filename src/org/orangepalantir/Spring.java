package org.orangepalantir;

import javax.imageio.ImageIO;
import java.awt.Color;
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
    private Color color;

    public Spring(Attachment a, Attachment b){
        this.a = a;
        this.b = b;
        setColor(new Color(255, 255, 255, 155));
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

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }
}