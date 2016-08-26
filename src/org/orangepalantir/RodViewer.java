package org.orangepalantir;

import javax.swing.JFrame;
import javax.swing.JPanel;
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

/**
 * Created by melkor on 7/31/16.
 */
public class RodViewer {
    List<DrawableRod> rods = new ArrayList<>();
    List<Spring> springs = new ArrayList<>();

    RigidRod selected;
    BufferedImage display;
    String status = "";
    int width = 512;
    int height = 512;
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

    double simWidth = 3.0;
    double simHeight = 3.0;

    public RodViewer(){

    }
    public void addRod(DrawableRod rod){
        rods.add(rod);
    }

    public void buildGui(){
        JFrame frame= new JFrame();
        JPanel outer = new JPanel();
        outer.setLayout(new BorderLayout());
        outer.add(panel, BorderLayout.CENTER);
        outer.add(slider, BorderLayout.SOUTH);
        frame.setContentPane(outer);
        frame.pack();
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
        g2d.fillRect(0, 0, width, height);
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        double[] Axy = new double[2];
        double[] Bxy = new double[2];
        g2d.setColor(Color.BLACK);
        g2d.drawString(status, 50, 50);





        float width = 10f;
        g2d.setStroke(new BasicStroke(width, BasicStroke.JOIN_ROUND, BasicStroke.CAP_ROUND));
        for(DrawableRod rod: rods){
            List<Point> points = rod.getPoints();
            for(int i = 0;i<points.size()-1; i++){

                getTransformed(points.get(i), Axy);
                getTransformed(points.get(i+1), Bxy);

                double dx = Bxy[0] - Axy[0];
                double dy = Bxy[1] - Axy[1];
                double l = Math.sqrt(dx*dx + dy*dy);

                double cx = 0.5*(Bxy[0] + Axy[0]);
                double cy = 0.5*(Bxy[1] + Axy[1]);

                g2d.setPaint(new GradientPaint(
                        (float)(cx + width/2*dy/l),
                        (float)(cy - width/2*dx/l),
                        Color.RED,
                        (float)cx,
                        (float)cy,
                        Color.WHITE,
                        true
                ));

                g2d.drawLine((int)Axy[0], (int)Axy[1],(int)Bxy[0], (int)Bxy[1]);


            }



        }

        for(int i = 0; i<springs.size(); i++){
            Spring s = springs.get(i);
            getTransformed(s.a.getAttachment(), Axy);
            getTransformed(s.b.getAttachment(), Bxy);
            g2d.setColor(Color.CYAN);
            g2d.setStroke(new BasicStroke(5f, BasicStroke.JOIN_ROUND, BasicStroke.CAP_ROUND));
            g2d.drawLine((int)Axy[0], (int)Axy[1],(int)Bxy[0], (int)Bxy[1]);
        }


        display = drawing;
        panel.repaint();
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

    void addSpring(Spring s){
        springs.add(s);
    }
}
