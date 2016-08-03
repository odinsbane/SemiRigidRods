package org.orangepalantir;

/**
 * Created by melkor on 8/2/16.
 */
public class Vector{

    public Vector(Point a, Point b){
        dx = b.x - a.x;
        dy = b.y - a.y;
        dz = b.z - a.z;
        length = Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    /**
     *
     * @param dx
     * @param dy
     * @param dz
     */
    public Vector(double dx, double dy, double dz){
       this.dx = dx;
        this.dy = dy;
        this.dz = dz;
        length = Math.sqrt(dx*dx + dy*dy + dz*dz);
    }
    /**
     * Creates a new vector with the corresponding magnitude in the same direction.
     *
     * @param magnitude length
     * @param direction direction
     */
    public Vector(double magnitude, Vector direction){
        dx = direction.dx*magnitude;
        dy = direction.dy*magnitude;
        dz = direction.dz*magnitude;
        length = magnitude;
    }

    double  dx, dy, dz;
    double length;
    void normalize(){

        dx = dx/length;
        dy = dy/length;
        dz = dz/length;
        length = 1;
    }

    /**
     * returns a vector projected along this vector. Both vectors should be normalized.
     *
     * @param other
     * @return
     */
    public Vector projection(Vector other){
        double dot = other.dx*dx + other.dy*dy + other.dz*dz;
        Vector a = new Vector(dot, this);
        return a;
    }

    /**
     * returns a vector that is this vector minus the arguments.
     *
     */
    public Vector minus(Vector other){
        return new Vector(dx - other.dx, dy - other.dy, dz - other.dz);
    }
}
