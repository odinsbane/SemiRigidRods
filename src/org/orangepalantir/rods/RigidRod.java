package org.orangepalantir.rods;

import org.orangepalantir.rods.integrators.UpdatableAgent;

import java.awt.EventQueue;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Created by melkor on 7/31/16.
 */
public class RigidRod implements DrawableRod, UpdatableAgent {
    double[] appliedForces;
    public double[] totalForces;
    public Point[] points;
    public double k = 100;
    public double kappa = 0.0166;
    final public int N;
    public double length;

    double Kspring;
    double Kbend;
    double Ktor = 0;
    double ds0;

    public RigidRod(Point[] points, double springStiffness, double bendingStiffness, double length){
        this.points = points;
        N = points.length;
        ds0 = length/(N-1);
        this.length = length;
        k = springStiffness;
        kappa = bendingStiffness;
        Kspring = k/ds0;
        Kbend = kappa/(ds0*ds0);
        appliedForces = new double[3*N];
        totalForces = new double[3*N];

    }

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

    /**
     * Find all the intesections of a sphere and a this rigid rod. Points will be found by
     * @param center
     * @param radius
     * @return
     */
    public double[] getIntersections(Point center, double radius){


        double[] intersections = new double[5];
        int count = 0;
        for(int i = 0; i<points.length-1; i++){
            Point first = points[i];
            Point second = points[i+1];
            Vector v = new Vector(first, second);
            Vector t = v.normalize();
            Vector r = new Vector(center, first);

            Vector rparallel = v.projection(r);
            Vector rperp = r.minus(rparallel);
            if(rperp.length>radius){
                return new double[0];
            }

            double along = Math.sqrt(radius*radius - rperp.length*rperp.length);

            double dot = rparallel.dot(t);
            double forward = dot + along;
            double backward = dot - along;

            if(forward>0 && forward<v.length){
                if(count==intersections.length){
                    //what the hell?
                    double[] ni = new double[intersections.length*2];
                    System.arraycopy(intersections, 0, ni, 0, count);
                }

                intersections[count] = forward + i*ds0;
                count++;

            }

            if(backward>0 && backward<=v.length){
                if(count==intersections.length){
                    //what the hell?
                    double[] ni = new double[intersections.length*2];
                    System.arraycopy(intersections, 0, ni, 0, count);
                }

                intersections[count] = backward + i*ds0;
                count++;
            }

        }

        return Arrays.copyOf(intersections, count);
    }
    public void applyForce(double fx, double fy, double fz, double location){
        double full = (location + length*0.5)/ds0;
        int dex = (int)(full);
        full = full - dex;

        if(dex>=N-1){
            dex=N-1;
            full = 0;
        } else if(dex<0){
            dex = 0;
            full = 0;
        }
        double fb = full;
        double fa =  1 -full;

        dex = 3*dex;
        appliedForces[dex] += fx*fa;
        appliedForces[dex + 1] += fy*fa;
        appliedForces[dex + 2] += fz*fa;

        if(full>0){
            appliedForces[dex + 3] += fx*fb;
            appliedForces[dex + 4] += fy*fb;
            appliedForces[dex + 5] += fz*fb;
        }
    }

    public int[] getIndexesOfClosestApproach(RigidRod b, double width){
        double[] x1 = new double[3];
        double[] x2 = new double[3];

        double min = Double.MAX_VALUE;
        int[] dexes = { -1, -1};
        for(int i = 0; i<points.length; i++){
            Point p = points[i];
            for(int j = 0; j<b.points.length; j++){
                Point o = b.points[j];
                p.getPosition(x1);
                o.getPosition(x2);
                wrap(x1, x2, width);
                double d = distSqd(x1, x2);
                if(d<min){
                    min = d;
                    dexes[0] = i;
                    dexes[1] = j;
                }
            }
        }
        return dexes;
    }
    public double getClosestApproach(RigidRod b, double width){
        double[] x1 = new double[3];
        double[] x2 = new double[3];

        double min = Double.MAX_VALUE;
        for(Point p: points){
            for(Point o: b.points){
                p.getPosition(x1);
                o.getPosition(x2);
                wrap(x1, x2, width);
                double d = distSqd(x1, x2);
                if(d<min){
                    min = d;
                }
            }
        }
        return Math.sqrt(min);
    }

    public static double distSqd(double[] a, double[] b){
        double sum = 0;
        double delta = b[0] - a[0];
        sum += delta*delta;
        delta = b[1] - a[1];
        sum += delta*delta;
        delta = b[2] - a[2];
        sum += delta*delta;
        return sum;
    }



    final static double attempts = 2;
    public void step(double dt){
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
        //smearForces(appliedForces, totalForces);
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
        double r0p = t0.length/ds0;
        double r1p, t0t1;
        double fa, fb;
        for(int i = 1; i<N-1; i++){

            Point current = points[i];
            Point front = points[i+1];

            t1 = new Vector(current, front);
            r1p = t1.length/ds0;

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
            t0t1 = (t0.dx*t1.dx + t0.dy*t1.dy + t0.dz*t1.dz);

            fa = Kbend*t0t1/r0p*t0.dx - Kbend/r0p*t1.dx;
            fb = Kbend/r1p*t0.dx - Kbend*t0t1*t1.dx;

            totalForces[low] += fa;
            totalForces[high] += fb;
            totalForces[mid] -= (fa + fb);

            fa = Kbend*t0t1/r0p*t0.dy - Kbend/r0p*t1.dy;
            fb = Kbend/r1p*t0.dy - Kbend*t0t1*t1.dy;
            totalForces[low+1] += fa;
            totalForces[high+1] += fb;
            totalForces[mid+1] -= (fa + fb);

            fa = Kbend*t0t1/r0p*t0.dz - Kbend/r0p*t1.dz;
            fb = Kbend/r1p*t0.dz - Kbend*t0t1*t1.dz;
            totalForces[low+2] += fa;
            totalForces[high+2] += fb;
            totalForces[mid+2] -= (fa + fb);

            t0 = t1;
            r0p = r1p;
        }
        double sum = 0;
        double v;
        for(int i = 0; i<totalForces.length; i++){
            v = totalForces[i];
            sum += v*v;
        }
        return Math.sqrt(sum);
    }



    public double calculateStretchEnergy(){
        double energy = 0;
        for(int i = 1; i<N; i++){
            Point a = points[i-1];
            Point b = points[i];
            Vector t0 = new Vector(a, b);
            energy += 0.5*sq(t0.length-ds0)*Kspring;
        }


        return energy;

    }


    static double wrap(double origin, double target, double width){
        double delta = target - origin;
        return  delta > width/2 ? target - width :
                delta < -width/2 ? target + width : target;
    }

    /**
     * Modifies the target to be the closest position to the origin when using reflected coordinates.
     *
     * @param origin
     * @param target
     * @param width
     * @return
     */
    static void wrap(double[] origin, double[] target, double width){
        target[0] = wrap(origin[0], target[0], width);
        target[1] = wrap(origin[1], target[1], width);
        target[2] = wrap(origin[2], target[2], width);
    }

    public double calculateBendEnergy(){
        double energy = 0;
        Vector t0 = new Vector(points[0], points[1]);
        for(int i = 1; i<N-1; i++){
            Point b = points[i];
            Point c = points[i+1];
            Vector t1 = new Vector(b, c);

            energy += 0.5*ds0*Kbend*(1 - t0.dx*t1.dx - t0.dy*t1.dy - t0.dz*t1.dz);
            t0 = t1;
        }


        return energy;

    }


    double sq(double v){return v*v;}

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
        double c = Math.sqrt(dx*dx + dy*dy + dz*dz)/ds0;
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

    public double getSFromIndex(int dex){

        return dex*length/points.length - length/2;

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

    public static void main(String[] args){
        RigidRod r0 = new RigidRod(new Point(0, 0, 0), new Vector(1, 0, 0), 51, 2);
        double f = 0.005;
        double c = -0.0;
        r0.applyForce(c, f, 0, -1.0);
        r0.applyForce(-c, f, 0, 1.0);
        r0.applyForce(0, -2*f, 0, 0);

        AnalyticBentRod br = new AnalyticBentRod(2, 1*f/r0.kappa);


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

    public void setStiffness(double v) {
        this.k = v;
        Kspring = v/ds0;
    }
}

