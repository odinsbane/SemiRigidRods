package org.orangepalantir;

/**
 * Created by melkor on 8/2/16.
 */
public class Point{
    double x, y, z;
    public Point(double x, double y, double z){
        this.x=x;
        this.y=y;
        this.z=z;
    }
    public Point(double[] xyz){
        this.x = xyz[0];
        this.y = xyz[1];
        this.z = xyz[2];
    }

    /**
     * Copies 3 coordinates into the provided array.
     *
     * @param a puts x,y,z into the 0th, 1st, and 2nd indexes.
     */
    public void getPosition(double[] a){
        a[0] = x;
        a[1] = y;
        a[2] = z;
    }

    /**
     * Copies 3 coordinates into the provided array.
     *
     * @param a puts x,y,z into the offset, offset+1, offset + 2 indexes.
     * @param offset starting index for coordinate storage.
     */
    public void getPosition(double[] a, int offset){
        a[offset] = x;
        a[offset+1] = y;
        a[offset+2] = z;
    }
}
