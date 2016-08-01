package org.orangepalantir;

import java.awt.EventQueue;

/**
 * Created by melkor on 7/31/16.
 */
public class RigidRod {
    double[] appliedForces;
    double[] totalForces;
    Point[] points;
    double k = 1000;
    double kappa = 0.0166;

    double Kspring;
    double Kbend;
    double Ktor = 0;
    int N;
    double length;
    double ds0;
    double dt=1e-5;

    public RigidRod(Point center, Vector direction, int N, double length){
        this.N = N;
        this.length = length;
        points = new Point[N];
        appliedForces = new double[3*N];
        totalForces = new double[3*N];
        ds0 = length/(N-1);
        Kspring = k/ds0;
        Kbend = kappa/(ds0*ds0);

        for(int i = 0; i<N; i++){

            double x = center.x + (ds0*i - length*0.5)*direction.dx;
            double y = center.y + (ds0*i - length*0.5)*direction.dy;
            double z = center.z + (ds0*i - length*0.5)*direction.dz;
            points[i] = new Point(x, y, z);

        }

    }

    public void applyForce(double fx, double fy, double fz, double location){
        int dex = (int)((location + length*0.5)/ds0);
        if(dex>=N){
            dex=N-1;
        } else if(dex<0){
            dex = 0;
        }
        Point p = points[dex];



        appliedForces[3*dex] += fx;
        appliedForces[3*dex + 1] += fy;
        appliedForces[3*dex + 2] += fz;

    }

    public double relax(){
        System.arraycopy(appliedForces, 0, totalForces, 0, 3*N);
        prepareInternalForces();
        return step();
    }
    private double step(){
        double sum = 0;
        for(int i = 0; i<N; i++){
            Point a = points[i];
            int dex = 3*i;

            a.x += totalForces[dex]*dt;
            a.y += totalForces[dex+1]*dt;
            a.z += totalForces[dex+2]*dt;

            sum += totalForces[dex]*totalForces[dex]
                    + totalForces[dex+1]*totalForces[dex+1]
                    + totalForces[dex+2]*totalForces[dex+2];
        }
        return Math.sqrt(sum);

    }

    private void prepareInternalForces(){
        Vector t0 = new Vector(points[0], points[1]);

        //apply spring for to first two points.
        double fMag = Kspring*(t0.length - ds0)/t0.length;
        int high = 3;
        int mid = 0;
        int low;

        double f = t0.dx*fMag;
        totalForces[high] += -f;
        totalForces[mid] += f;

        f = t0.dy*fMag;
        totalForces[high+1] += -f;
        totalForces[mid+1] += f;

        f = t0.dz*fMag;
        totalForces[high+2] += -f;
        totalForces[mid+2] += f;

        t0.normalize();

        Vector t1;
        for(int i = 1; i<N-1; i++){
            Point back = points[i-1];
            Point current = points[i];
            Point front = points[i+1];

            t1 = new Vector(current, front);

            //only apply spring force forward to current.
            fMag = (t1.length - ds0)/t1.length*Kspring;
            high = 3*(i+1);
            mid = 3*i;
            low = 3*(i-1);

            f = t1.dx*fMag;
            totalForces[high] += -f;
            totalForces[mid] += f;

            f = t1.dy*fMag;
            totalForces[high+1] += -f;
            totalForces[mid+1] += f;

            f = t1.dz*fMag;
            totalForces[high+2] += -f;
            totalForces[mid+2] += f;

            t1.normalize();


            //bending forces
            Vector t2 = new Vector(back, front);
            t2.normalize();

            Vector projection = t2.projection(t0);

            Vector bend = t0.minus(projection);

            fMag = Kbend;
            f=fMag*bend.dx*0.5;
            totalForces[low] += f;
            totalForces[high] += f;
            totalForces[mid] -= 2*f;

            f=fMag*bend.dy*0.5;
            totalForces[low+1] += f;
            totalForces[high+1] += f;
            totalForces[mid+1] -= 2*f;

            f=fMag*bend.dz*0.5;
            totalForces[low+2] += f;
            totalForces[high+2] += f;
            totalForces[mid+2] -= 2*f;

            t0 = t1;
        }
    }

    public static void main(String[] args){
        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 21, 2);
        double f = 0.025;
        double c = 0.01;
        r0.applyForce(c, f, 0, -1.0);
        r0.applyForce(-c, f, 0, 1.0);
        r0.applyForce(0, -2*f, 0, 0);

        RigidRod r1 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 51, 2);
        r1.applyForce(c, f, 0, -1.0);
        r1.applyForce(-c, f, 0, 1.0);
        r1.applyForce(0, -2*f, 0, 0);

        RigidRod r2 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 11, 2);
        r2.applyForce(c, f, 0, -1.0);
        r2.applyForce(-c, f, 0, 1.0);
        r2.applyForce(0, -2*f, 0, 0);


        RodViewer viewer = new RodViewer();
        viewer.addRod(r0);
        viewer.addRod(r1);
        viewer.addRod(r2);
        EventQueue.invokeLater(viewer::buildGui);
        while(viewer.displays()){
            double s = 0;
            for(int j = 0; j<1000; j++){
                s = r0.relax();
                s += r1.relax();
                s += r2.relax();
            }
            viewer.setStatus("" + s);
            viewer.repaint();
        }

    }


}

class Point{
    double x, y, z;
    public Point(double x, double y, double z){
        this.x=x;
        this.y=y;
        this.z=z;
    }
}

class Vector{

    Vector(Point a, Point b){
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
    Vector(double dx, double dy, double dz){
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
    Vector(double magnitude, Vector direction){
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
    Vector projection(Vector other){
        double dot = other.dx*dx + other.dy*dy + other.dz*dz;
        Vector a = new Vector(dot, this);
        return a;
    }

    /**
     * returns a vector that is this vector minus the arguments.
     *
     */
    Vector minus(Vector other){
        return new Vector(dx - other.dx, dy - other.dy, dz - other.dz);
    }
}