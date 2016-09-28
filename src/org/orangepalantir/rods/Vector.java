package org.orangepalantir.rods;

/**
 * Created by melkor on 8/2/16.
 */
public class Vector{
    public static final Vector ZERO = new Vector(0,0,0);

    public double  dx, dy, dz;
    public double length;

    public Vector(Point a, Point b){
        dx = b.x - a.x;
        dy = b.y - a.y;
        dz = b.z - a.z;
        length = Math.sqrt(dx*dx + dy*dy + dz*dz);

        if(length==0){
            //why?
            this.dx = 0;
            this.dy = 0;
            this.dz = 0;

        } else {
            dx = dx / length;
            dy = dy / length;
            dz = dz / length;
        }


    }

    /**
     *
     * @param dx
     * @param dy
     * @param dz
     */
    public Vector(double dx, double dy, double dz){
        length = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if(length==0){

            this.dx = 0;
            this.dy = 0;
            this.dz = 0;

        } else {
            this.dx = dx / length;
            this.dy = dy / length;
            this.dz = dz / length;
        }

    }
    /**
     * Creates a new vector with the corresponding magnitude in the same direction.
     *
     * @param magnitude length
     * @param direction direction
     */
    public Vector(double magnitude, Vector direction){

        dx = direction.dx;
        dy = direction.dy;
        dz = direction.dz;

        length = magnitude;
    }

    /**
     * returns a vector projected along the direction of this vector.
     *
     * @param other
     * @return
     */
    public Vector projection(Vector other){
        double dot = other.dx*dx + other.dy*dy + other.dz*dz;
        Vector a = new Vector(dot*other.length, this);
        return a;
    }

    /**
     * returns a vector that is this vector minus the arguments.
     *
     */
    public Vector minus(Vector other){

        return new Vector(dx*length - other.dx*other.length, dy*length - other.dy*other.length, dz*length - other.dz*other.length);

    }

    /**
     * returns a vector that is this vector minus the arguments.
     *
     */
    public Vector add(Vector other){

        return new Vector(dx*length + other.dx*other.length, dy*length + other.dy*other.length, dz*length + other.dz*other.length);

    }

    /**
     * Sets the length to 1, unless the length is already 0.
     */
    public void normalize(){
        length = length==0?0:1;
    }


}
