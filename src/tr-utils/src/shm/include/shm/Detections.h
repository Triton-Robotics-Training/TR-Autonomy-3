#ifndef DETECTIONS_H
#define DETECTIONS_H

// Define leaf structs first (no dependencies)
struct BoundingBox2D {
    float center_x;
    float center_y;
    float size_x;
    float size_y;
};

struct PoseWithCovariance {
    float pose[3];
    float covariance[36];
};

// Finally, structs with flexible array members
struct Detection2D {
    BoundingBox2D bbox;
    float score;
};

struct Detection2DArray {
    int num_detections;
    long timestamp; // time the detection was written
    // POD assumes a maximum of 10 detections
    Detection2D detections[10];
    // each detection includes 2 light bars
    BoundingBox2D light_bar_pairs[10][2];
};

#endif // DETECTIONS_H