package org.orangepalantir;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Created by msmith on 18/08/16.
 */
public class ImplicitRod implements DrawableRod {

    double[] appliedForces;
    double[][] connections;

    public double[] totalForces;
    public Point[] points;
    double k = 100;
    double kappa = 0.0166;

    double Kspring;
    double Kbend;
    double Ktor = 0;
    final public int N;
    double length;
    double ds0;
    double dt=1e-4;

    public ImplicitRod(Point center, Vector direction, int N, double length){
        this.N = N;
        connections = new double[N][N];
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
    public void clearForces(){
        Arrays.fill(appliedForces, 0.0);
    }

    public void applyForce(double fx, double fy, double fz, double location){
        int dex = (int)((location + length*0.5)/ds0);
        if(dex>=N){
            dex=N-1;
        } else if(dex<0){
            dex = 0;
        }

        appliedForces[3*dex] += fx;
        appliedForces[3*dex + 1] += fy;
        appliedForces[3*dex + 2] += fz;

    }

    public double relax(){
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

    public void step(double dt){
        for(int i = 0; i<N; i++){
            Point a = points[i];
            int dex = 3*i;

            a.x += totalForces[dex]*dt;
            a.y += totalForces[dex+1]*dt;
            a.z += totalForces[dex+2]*dt;

        }
    }

    public double outOfEq(){
        double sum = 0;
        for(double f: totalForces){
            sum += f*f;
        }
        return Math.sqrt(sum);
    }

    /**
     * Gets an estimate of the position and direction.
     *
     * @param pos
     * @param dir
     */
    public void getPositionAndDirection(double[] pos, double[] dir){
        Point center = points[N/2];
        center.getPosition(pos);
        Point end = points[N-1];
        end.getPosition(dir);
        dir[0] = dir[0] - pos[0];
        dir[1] = dir[1] - pos[1];
        dir[2] = dir[2] - pos[2];
        double l = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2];
        if(l>0){
            l = 1.0/Math.sqrt(l);
            dir[0] *= l;
            dir[1] *= l;
            dir[2] *= l;
        }
    }

    public void prepareInternalForces(){
        System.arraycopy(appliedForces, 0, totalForces, 0, 3*N);
        Vector t0 = new Vector(points[0], points[1]);

        //apply spring force to first two points.
        double fMag = Kspring*(t0.length - ds0);
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

        Vector t1;
        for(int i = 1; i<N-1; i++){

            Point current = points[i];
            Point front = points[i+1];

            t1 = new Vector(current, front);

            //only apply spring force forward to current.
            fMag = (t1.length - ds0)*Kspring;
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


            //bending forces
            double t2x = t1.dx + t0.dx;
            double t2y = t1.dy + t0.dy;
            double t2z = t1.dz + t0.dz;

            Vector t2 = new Vector(t2x, t2y, t2z);

            Vector projection = t2.projection(t0);

            Vector bend = t0.minus(projection);

            fMag = Kbend*bend.length;

            f=fMag*bend.dx;
            totalForces[low] += f;
            totalForces[high] += f;
            totalForces[mid] -= 2*f;

            f=fMag*bend.dy;
            totalForces[low+1] += f;
            totalForces[high+1] += f;
            totalForces[mid+1] -= 2*f;

            f=fMag*bend.dz;
            totalForces[low+2] += f;
            totalForces[high+2] += f;
            totalForces[mid+2] -= 2*f;

            t0 = t1;
        }
    }

    @Override
    public List<Point> getPoints() {
        return Arrays.stream(points).collect(Collectors.toList());
    }

    /**
     * Finds the closest point along the curve. s should lie between two points, and the x,y,z of an
     * interpolated position will be returned.
     *
     * @param s
     * @param p
     * @return
     */
    public void getPoint(double s, double[] p){
        double dexter = (s + length*0.5)/ds0;
        if(dexter>=N-1){
            //dex=N-1;
            points[N-1].getPosition(p);
            return;
        } else if(dexter<=0){
            points[0].getPosition(p);
            return;
        }
        double f = dexter - (int)dexter;
        int low = (int)dexter;
        int high= low + 1;

        points[low].getPosition(p);
        double x = p[0];
        double y = p[1];
        double z = p[2];

        points[high].getPosition(p);

        p[0] = x + (p[0]-x)*f;
        p[1] = y + (p[1]-y)*f;
        p[2] = z + (p[2]-z)*f;

    }

    public void setPositions(double[] data, int offset){
        for(int i = 0; i<points.length; i++){
            int current = i*3 + offset;
            Point p = points[i];
            p.x = data[current];
            p.y = data[current+1];
            p.z = data[current+2];
        }
    }

    public Vector getTangent(double loc) {

        double dexter = (loc + length*0.5)/ds0;
        if(dexter>=N-1){
            //dex=N-1;
            Vector v = new Vector(points[N-2], points[N-1]);
            return v;
        } else if(dexter<=0){
            Vector v = new Vector(points[0], points[1]);
            return v;
        }
        double f = dexter - (int)dexter;
        int low = (int)dexter;
        int high= low + 1;

        Vector v = new Vector(points[low], points[high]);
        return v;
    }

}
