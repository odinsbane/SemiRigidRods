package org.orangepalantir.rods.io;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.integrators.AdaptiveIntegrator;
import org.orangepalantir.rods.interactions.FixedForceAttachment;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.stream.Collectors;

/**
 * For loading and saving rods.
 *
 * Created by Matt on 28/09/16.
 */
public class RodIO implements AutoCloseable{
    final static int SAVE = 0;
    final static int LOAD = 1;
    final int mode;
    final static int RIGID_ROD = 0;
    final static int CROSSLINKER = 1;
    final static int MYOSIN = 2;
    final static int FIXED_FORCE=3;
    List<RigidRod> rods;
    List<Spring> springs;
    List<Motor> motors = new ArrayList<>();
    DataOutputStream output;
    DataInputStream input;
    double width = 10;
    long time;

    private RodIO(int mode){
        this.mode = mode;
    }


    private void loadData() throws IOException {
        int read;
        time = input.readLong();
        width = input.readDouble();
        System.out.println("width: " + width);
        do{
            try {
                read = input.readInt();
            } catch(Exception exc){
                //just return what you have.
                read = -1;
            }
            switch(read){
                case RIGID_ROD:
                    rods.add(readRigidRod());
                    break;
                case CROSSLINKER:
                    springs.add(readCrossLinker());
                    break;
                case MYOSIN:
                    motors.add(readMotor());
                    break;
                case FIXED_FORCE:
                    springs.add(readFixedForce());
                    break;
                default:
                    //oh well.
            }

        }while(read>=0);

    }

    private void writeCrosslinker(Spring s) throws IOException {
        output.writeInt(CROSSLINKER);
        output.writeDouble(s.getRestLength());
        output.writeDouble(s.getStiffness());
        RigidRodAttachment a = (RigidRodAttachment) s.a;
        RigidRodAttachment b = (RigidRodAttachment) s.b;
        output.writeInt(rods.indexOf(a.rod));
        output.writeDouble(a.loc);
        output.writeInt(rods.indexOf(b.rod));
        output.writeDouble(b.loc);

    }

    private Spring readCrossLinker() throws IOException {
        double l = input.readDouble();
        double k = input.readDouble();
        int a = input.readInt();
        double as = input.readDouble();
        int b = input.readInt();
        double bs = input.readDouble();
        RigidRodAttachment aa = new RigidRodAttachment(as, rods.get(a));
        RigidRodAttachment ba = new RigidRodAttachment(bs, rods.get(b));
        Spring s = new Spring(aa, ba, width);
        s.setRestLength(l);
        s.setStiffness(k);
        return s;
    }



    private Motor readMotor() throws IOException {
        double stalkLength = input.readDouble();
        double stalkStiffness = input.readDouble();
        double springLength = input.readDouble();
        double springStiffness = input.readDouble();
        double bindTau = input.readDouble();

        Motor m = new Motor(stalkLength, stalkStiffness, springLength, springStiffness, bindTau, width);

        //front.
        double[] head = new double[]{input.readDouble(), input.readDouble(), input.readDouble()};
        m.setPosition(Motor.FRONT, head);
        double[] tail = {input.readDouble(), input.readDouble(), input.readDouble()};
        m.setPosition(Motor.BACK, tail);

        int aDex = input.readInt();
        if(aDex>=0) {
            double aLoc = input.readDouble();
            double aTime = input.readDouble();
            m.bindRod(rods.get(aDex), aLoc, Motor.FRONT, aTime);
        }

        int bDex = input.readInt();

        if(bDex>=0) {
            double bLoc = input.readDouble();
            double bTime = input.readDouble();
            m.bindRod(rods.get(bDex), bLoc, Motor.BACK, bTime);
        }
        return m;

    }

    private void write(Motor motor) throws IOException{
        output.writeInt(MYOSIN);
        output.writeDouble(motor.stalkLength);
        output.writeDouble(motor.stalkStiffness);
        output.writeDouble(motor.springLength);
        output.writeDouble(motor.springStiffness);
        output.writeDouble(motor.getBindTau());
        List<Point> points = motor.getPoints();
        //front.
        output.writeDouble(points.get(Motor.FRONT).x);
        output.writeDouble(points.get(Motor.FRONT).y);
        output.writeDouble(points.get(Motor.FRONT).z);

        //back.
        output.writeDouble(points.get(Motor.BACK).x);
        output.writeDouble(points.get(Motor.BACK).y);
        output.writeDouble(points.get(Motor.BACK).z);
        RigidRodAttachment a = motor.getBound(Motor.FRONT);
        if(a==null){
            output.writeInt(-1);
        }else {
            output.writeInt(rods.indexOf(a.rod));
            output.writeDouble(a.loc);
            output.writeDouble(motor.getTimeRemaining(Motor.FRONT));
        }
        RigidRodAttachment b = motor.getBound(Motor.BACK);
        if(b==null){
            output.writeInt(-1);
        } else{
            output.writeInt(rods.indexOf(b.rod));
            output.writeDouble(b.loc);
            output.writeDouble(motor.getTimeRemaining(Motor.BACK));
        }
    }



    private RigidRod readRigidRod() throws IOException {
        double l = input.readDouble();
        double k = input.readDouble();
        double kappa = input.readDouble();
        int N = input.readInt();
        Point[] points = new Point[N];
        for(int i = 0; i<N; i++){
            points[i] = new Point(input.readDouble(), input.readDouble(), input.readDouble());
        }

        return new RigidRod(points, k, kappa, l);
    }

    private void write(RigidRod rod) throws IOException {
        output.writeInt(RIGID_ROD);
        output.writeDouble(rod.length);
        output.writeDouble(rod.k);
        output.writeDouble(rod.kappa);
        output.writeInt(rod.N);
        for(Point p: rod.getPoints()){
            output.writeDouble(p.x);
            output.writeDouble(p.y);
            output.writeDouble(p.z);
        }
    }


    private void write(Spring spring) throws IOException {
        if(spring.a instanceof FixedForceAttachment){
            writeFixedForce(spring);
        } else if(spring.a instanceof RigidRodAttachment && spring.b instanceof RigidRodAttachment){
            writeCrosslinker(spring);
        }
    }


    private Spring readFixedForce() throws IOException {
        int dex = input.readInt();
        RigidRod r = rods.get(dex);
        double loc = input.readDouble();
        double fx = input.readDouble();
        double fy = input.readDouble();
        double fz = input.readDouble();
        FixedForceAttachment ffa = new FixedForceAttachment(r, loc, new double[]{fx, fy, fz});
        return new Spring(ffa, ffa.getDanglingEnd(), width);
    }

    private void writeFixedForce(Spring s) throws IOException {
        FixedForceAttachment at = (FixedForceAttachment)s.a;
        int dex = rods.indexOf(at.getRod());
        double ap = at.getAttachmentLocation();
        double[] f = new double[3];
        at.getActualForce(f);
        output.writeInt(FIXED_FORCE);
        output.writeInt(dex);
        output.writeDouble(ap);
        output.writeDouble(f[0]);
        output.writeDouble(f[1]);
        output.writeDouble(f[2]);
    }


    /**
     * For work with a path, uses
     * @param path
     * @throws IOException
     */
    public void setOutput(Path path) throws IOException {
        setOutput(new DataOutputStream(Files.newOutputStream(path)));
    }

    public void setOutput(DataOutputStream out){
        output = out;
    }

    public void setWidth(double w){
        width = w;
    }

    public void write() throws IOException{
        if(mode==LOAD) throw new IOException("RodIO not opened for saving.");

        long time = System.currentTimeMillis();
        output.writeLong(time);
        output.writeDouble(width);
        for(RigidRod rod: rods){
            write(rod);
        }
        for(Motor motor: motors){
            write(motor);
        }

        for(Spring spring: springs){
            write(spring);
        }

        output.writeInt(-1);
    }

    @Override
    public void close() throws IOException {
        switch(mode){
            case SAVE:
                output.close();
                break;
            case LOAD:
                input.close();
                break;
        }
    }
    public static RodIO saveRigidRodSimulation() throws IOException {

        return new RodIO(SAVE);

    }

    public void setRods(List<RigidRod> rods) {
        this.rods = rods;
    }
    public void setMotors(List<Motor> motors){
        this.motors = motors;
    }
    public void setSprings(List<Spring> springs){
        this.springs = springs;
    }

    public static RodIO createRodLoader(){
        return new RodIO(LOAD);
    }
    public void loadRods(Path path) throws IOException {
        loadRigidRod(new DataInputStream(Files.newInputStream(path)));
    }

    public void loadRigidRod(DataInputStream input) throws IOException {
        this.input = input;
        setRods(new ArrayList<>());
        setSprings(new ArrayList<>());
        loadData();
    }

    public List<RigidRod> getRods() {
        return rods;
    }

    public List<Spring> getSprings() {
        return springs;
    }

    public static void main(String[] args){
        RigidRod rod = new RigidRod(new Point(0.5, 0.5, 0.5), new Vector(1, 0, 0), 3, 2);
        RigidRod rod2 = new RigidRod(new Point(0.5, 0.29, 0.5), new Vector(1, 0, 0), 3, 2);

        rod.setBendingStiffness(0.0166);
        rod.setStiffness(100);

        rod2.setBendingStiffness(0.0166);
        rod2.setStiffness(100);

        FixedForceAttachment force0 = new FixedForceAttachment(rod, -1, new double[]{0, 1, 0});
        FixedForceAttachment force1 = new FixedForceAttachment(rod, 1, new double[]{0, 1, 0});
        FixedForceAttachment force2 = new FixedForceAttachment(rod, 0, new double[]{0, -2, 0});
        List<Spring> springs = new ArrayList<>();
        springs.add(new Spring(force0, force0.getDanglingEnd(), 10));
        springs.add(new Spring(force1, force1.getDanglingEnd(), 10));
        springs.add(new Spring(force2, force2.getDanglingEnd(), 10));

        Spring link = new Spring(
                new RigidRodAttachment(0, rod),
                new RigidRodAttachment(0, rod2), 10);
        link.setRestLength(0.2);
        link.setStiffness(100);
        springs.add(link);


        List<RigidRod> rods = new ArrayList<>();
        rods.add(rod);
        rods.add(rod2);
        List<Motor> motors = new ArrayList<>();

        AdaptiveIntegrator integrator = new AdaptiveIntegrator();
        integrator.prepare(rods);

        double f = 0;
        for(int i = 0; i<1000; i++) {
            f = integrator.step(springs);
        }

        System.out.println(f);
        rods.forEach(RigidRod::clearForces);
        springs.forEach(Spring::applyForces);
        double sum = rods.stream(
        ).mapToDouble(
                RigidRod::prepareInternalForces
        ).sum();
        System.out.println(sum);

        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        DataOutputStream out = new DataOutputStream(buffer);

        try(RodIO writer = RodIO.saveRigidRodSimulation();){
            writer.setOutput(out);
            writer.setRods(rods);
            writer.setSprings(springs);
            writer.setWidth(10);
            writer.write();
        } catch (IOException e) {
            e.printStackTrace();
        }
        byte[] bytes = buffer.toByteArray();
        DataInputStream in = new DataInputStream(new ByteArrayInputStream(bytes));
        List<RigidRod> rods2 = new ArrayList<>();
        List<Spring> springs2 = new ArrayList<>();
        try(RodIO reader = RodIO.createRodLoader()){
            reader.loadRigidRod(in);
            rods2.addAll(reader.getRods());
            springs2.addAll(reader.getSprings());
        } catch (Exception e){
            e.printStackTrace();
        }
        System.out.println(rods2.size() + " rods and, " + springs2.size() + " springs");
        springs2.forEach(Spring::applyForces);
        rods2.forEach(r->{System.out.println(r.prepareInternalForces());});

    }

    public double getWidth() {
        return width;
    }
}
