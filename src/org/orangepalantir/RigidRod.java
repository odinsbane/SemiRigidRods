package org.orangepalantir;

import java.awt.EventQueue;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Created by melkor on 7/31/16.
 */
public class RigidRod implements DrawableRod{
    double[] appliedForces;
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

    public void setBendingStiffness(double kappa2){
        kappa = kappa2;
        Kbend = kappa/(ds0*ds0);
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

    public void step(double dt){
        double attempts = 6;
        for(int j = 0; j<attempts; j++){
            for(int i = 0; i<N; i++) {
                Point a = points[i];
                int dex = 3 * i;

                a.x += totalForces[dex] * dt/attempts;
                a.y += totalForces[dex + 1] * dt/attempts;
                a.z += totalForces[dex + 2] * dt/attempts;
            }
            if(j<attempts-1) prepareInternalForces();
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

    public double prepareInternalForces(){
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
        double sum = 0;
        double v;
        for(int i = 0; i<totalForces.length; i++){
            v = totalForces[i];
            sum += v*v;
        }
        return Math.sqrt(sum);
    }

    public double calculateEnergy(){
        double energy = 0;

        for(int i = 1; i<N-1; i++){

        }


        return energy;

    }

    public double getMaxCurvature(){
        double max = 0;
        double sum = 0;
        for(int i = 1; i<points.length-1; i++){
            double c = getCurvature(i);
            sum += c;
            max = c>max?c:max;
        }
        max = sum/(points.length-2);

        return max;
    }

    public double getCurvature(int i){

        if(i<1||i>=points.length){
            return 0;
        }

        Vector t1 = new Vector(points[i-1], points[i]);
        Vector t2 = new Vector(points[i], points[i+1]);
        double dx = t2.dx - t1.dx;
        double dy = t2.dy - t1.dy;
        double dz = t2.dz - t1.dz;
        double c = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return c;
    }

    public double getMaxDisplacement(){
        double pos[] = new double[3];
        double dir[] = new double[3];
        getPositionAndDirection(pos, dir);
        double max = -1;
        for(int i = 0; i<points.length; i++){

            double s = i*ds0 - length/2;
            double x = pos[0] + s*dir[0];
            double y = pos[1] + s*dir[1];
            double z = pos[2] + s*dir[2];

            Point a = points[i];
            double dx = x - a.x;
            double dy = y - a.y;
            double dz = z - a.z;
            double d = Math.sqrt(dx*dx + dy*dy + dz*dz);
            max = max>d?max:d;
        }

        return max;
    }

    public static void main(String[] args){
        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 51, 2);
        double f = 0.0025;
        double c = 0.0;
        r0.applyForce(c, f, 0, -1.0);
        r0.applyForce(-c, f, 0, 1.0);
        r0.applyForce(0, -2*f, 0, 0);

        AnalyticBentRod br = new AnalyticBentRod(2, 2*f/r0.kappa);


        RodViewer viewer = new RodViewer();
        viewer.addRod(r0);
        viewer.addRod(br);
        viewer.setSelected(r0);
        double[] original = new double[r0.getValueCount()];
        double[] full = new double[r0.getValueCount()];
        double[] next = new double[r0.getValueCount()];
        EventQueue.invokeLater(viewer::buildGui);
        while(viewer.displays()){
            double s = 0;
            for(int j = 0; j<1000; j++){
                s=r0.prepareInternalForces();
                r0.step(0.001);
            }
            viewer.setStatus("" + s);
            viewer.repaint();
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

    public void restorePositions(double[] data, int offset){
        for(int i = 0; i<points.length; i++){
            int current = i*3 + offset;
            Point p = points[i];
            p.x = data[current];
            p.y = data[current+1];
            p.z = data[current+2];
        }
    }

    public void storePositions(double[] data, int offset){
        for(int i = 0; i<points.length; i++){
            Point p = points[i];
            int current = 3*i + offset;
            data[current] = p.x;
            data[current+1] = p.y;
            data[current+2] = p.z;
        }
    }

    public void storeForces(double[] data, int offset){
        System.arraycopy(totalForces, 0, data, offset, totalForces.length);
    }

    public void restoreForces(double[] data, int offset){
        System.arraycopy(data, offset, totalForces, 0, totalForces.length);
    }

    public int getValueCount(){
        return 3*N;
    }

    public int getForceCount(){
        return 3*N;
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

class AnalyticBentRod implements DrawableRod{
    List<Point> points = new ArrayList<>();

    double A,B;
    int N = 100;
    public AnalyticBentRod(double length, double force){
        A = force/6.0;
        B = force*length/4;
        double ds = length/(N-1);
        for(int i = 0; i<N; i++){
            double s = ds*i - length/2;
            points.add(new Point(s, getHeight(s), 0));
        }
    }

    double getHeight(double s){
        if(s>0){
            return -A*s*s*s + B*s*s;
        } else{
            return A*s*s*s + B*s*s;
        }
    }






    @Override
    public List<Point> getPoints() {
        return points;
    }

    @Override
    public void getPoint(double location, double[] pt) {
        //whatever.
    }
}