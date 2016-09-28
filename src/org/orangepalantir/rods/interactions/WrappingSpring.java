package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.interactions.Attachment;
import org.orangepalantir.rods.interactions.Spring;

/**
 * A spring whose vector is the shortest distance wrapped around a box of the provided
 * width.
 *
 * Created by msmith on 08/09/16.
 */
public class WrappingSpring extends Spring {
    double width;
    double[] points = new double[6];
    public WrappingSpring(Attachment a, Attachment b, double width){
        super(a, b);
        this.width = width;
    }

    @Override
    protected Vector getForce(){
        Point ptA = a.getAttachment();
        Point ptB = b.getAttachment();
        Vector v = getWrapped(ptA, ptB);

        double factor = v.length - s0;
        v.length = factor*k;
        return v;
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
