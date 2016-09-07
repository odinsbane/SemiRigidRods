package org.orangepalantir;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by melkor on 7/31/16.
 */
public class RodViewer {
    List<DrawableRod> rods = new ArrayList<>();
    List<Spring> springs = new ArrayList<>();

    RigidRod selected;
    public BufferedImage display;
    String status = "";
    int width = 800;
    int height = 800;
    Shape marker;
    Dimension a = new Dimension(width, height);

    JPanel panel = new JPanel(){

        protected void paintComponent(Graphics g){
            if(display!=null){
                g.drawImage(display, 0, 0, this);
            }
            if(marker!=null){
                ((Graphics2D)g).fill(marker);
            }
        }

        @Override
        public Dimension getPreferredSize(){
            return a;
        }
        @Override
        public Dimension getMaximumSize(){
            return a;
        }
        @Override
        public  Dimension getMinimumSize(){
            return a;
        }

    };
    JSlider slider = new JSlider();

    double simWidth = 10.0;
    double simHeight = 10.0;

    public RodViewer(){

    }
    public void addRod(DrawableRod rod){
        rods.add(rod);
    }

    public void buildGui(){
        JFrame frame= new JFrame();
        JPanel outer = new JPanel();
        outer.setLayout(new BorderLayout());
        outer.add(new JScrollPane(panel), BorderLayout.CENTER);
        outer.add(slider, BorderLayout.SOUTH);
        frame.setContentPane(outer);
        frame.setSize(640, 480);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
        slider.addChangeListener(evt->{
            if(selected!=null){
                double loc = (slider.getValue()-slider.getMinimum())*1.0/(slider.getMaximum() - slider.getMinimum());
                //loc goes from 0 to 1. we want it to go from -l/2 to +l/2
                loc = selected.length*(loc - 0.5);
                double[] pt = new double[3];
                selected.getPoint(loc, pt);
                double[] local = new double[2];
                getTransformed(new Point(pt), local);
                marker = new Ellipse2D.Double(local[0] - 3.5, local[1] - 3.5, 7, 7);
                panel.repaint();
            }
        });
    }

    public void setSelected(RigidRod rod){
        selected = rod;
    }

    public void repaint(){
        BufferedImage drawing = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = (Graphics2D)drawing.getGraphics();
        g2d.setColor(Color.BLACK);
        g2d.fillRect(0, 0, width, height);
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        double[] Axy = new double[2];
        double[] Bxy = new double[2];
        double[] shiftA = new double[2];
        double[] shiftB = new double[2];



        float width = 7f;
        double maxCurve = 0.5;
        Pattern pat = Pattern.compile("max-curvature: ([0-9.]*)");
        Matcher match = pat.matcher(status);
        if(match.find()){

            maxCurve = Double.parseDouble(match.group(1))*2;
            if(maxCurve==0){
                maxCurve = 1;
            }
        }
        g2d.setStroke(new BasicStroke(width, BasicStroke.JOIN_ROUND, BasicStroke.CAP_ROUND));
        Rectangle2D rect = new Rectangle2D.Double(0, 0, this.width, this.height);
        for(DrawableRod rod: rods){
            List<Point> points = rod.getPoints();
            for(int i = 0;i<points.size()-1; i++){

                Color color;
                if(rod instanceof RigidRod){
                    double c = ((RigidRod)rod).getCurvature(i);
                    c = c>maxCurve?maxCurve:c;
                    float f = (float)(c/maxCurve);
                    color = new Color(f, 1, f);
                } else{
                    color = Color.WHITE;
                }

                getTransformed(points.get(i), Axy);
                getTransformed(points.get(i+1), Bxy);

                double dx = Bxy[0] - Axy[0];
                double dy = Bxy[1] - Axy[1];
                double l = Math.sqrt(dx*dx + dy*dy);

                for(int tx = -1; tx<2; tx++){
                    for(int ty = -1; ty<2; ty++){
                        shiftA[0] = Axy[0] + tx*this.width;
                        shiftA[1] = Axy[1] + ty*this.width;
                        shiftB[0] = Bxy[0] + tx*this.width;
                        shiftB[1] = Bxy[1] + ty*this.width;

                        if(
                                (!rect.contains(shiftA[0], shiftA[1]))
                                        &&
                                (!rect.contains(shiftB[0], shiftB[1]))
                                                                       ){
                            continue;
                        }

                        double cx = 0.5*(shiftB[0] + shiftA[0]);
                        double cy = 0.5*(shiftB[1] + shiftA[1]);

                        g2d.setPaint(new GradientPaint(
                                (float)(cx + width/2*dy/l),
                                (float)(cy - width/2*dx/l),
                                Color.RED,
                                (float)cx,
                                (float)cy,
                                color,
                                true
                        ));

                        g2d.drawLine((int)shiftA[0], (int)shiftA[1],(int)shiftB[0], (int)shiftB[1]);
                    }
                }


            }



        }

        for(int i = 0; i<springs.size(); i++){
            Spring s = springs.get(i);

            Point a = s.a.getAttachment();
            Point b = getWrappedPoint(a, s.b.getAttachment());

            Vector v = new Vector(a, b);

            getTransformed(a, Axy);
            getTransformed(b, Bxy);

            g2d.setColor(s.getColor());
            g2d.setStroke(new BasicStroke(2f, BasicStroke.JOIN_ROUND, BasicStroke.CAP_ROUND));
            g2d.drawLine((int)Axy[0], (int)Axy[1],(int)Bxy[0], (int)Bxy[1]);
        }

        g2d.setColor(new Color(255, 255, 255, 200));
        g2d.fillRect(25,35, this.width-50, 20);
        g2d.setColor(Color.BLACK);
        g2d.drawString(status, 50, 50);

        display = drawing;
        panel.repaint();
    }
    double[] space = new double[6];
    public Point getWrappedPoint(Point a, Point b){
        a.getPosition(space);
        b.getPosition(space, 3);

        return new Point(
                space[0] + wrap(space[3] - space[0]),
                space[1] + wrap(space[4] - space[1]),
                space[2] + wrap(space[5] - space[2])
        );
    }
    double wrap(double delta){
        double hw = simWidth/2;
        return  delta > hw ? delta - simWidth :
                delta < -hw/2 ? delta + simWidth : delta;
    }
    void getTransformed(Point r, double[] tran){

        tran[0] = (r.x + simWidth/2) *width/simWidth;
        tran[1] = (r.y + simHeight/2)* height/simHeight;

    }


    public void setStatus(String s) {
        status = s;
    }

    public boolean displays() {
        return true;
    }

    public void addSpring(Spring s){
        springs.add(s);
    }

    public void setSimWidth(double width) {
        this.simWidth = width;
        this.simHeight = width;
    }
}
