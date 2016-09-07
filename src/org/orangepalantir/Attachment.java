package org.orangepalantir;

/**
 * Created by msmith on 05/09/16.
 */
public interface Attachment {
    Point getAttachment();
    default void applyForce(Vector v){}
}
