package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.Vector;

/**
 * Created by msmith on 05/09/16.
 */
public interface Attachment {
    Point getAttachment();
    default void applyForce(Vector v){}
}
