# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id

# 2D bounding box surrounding the object in pixels
yolov8/BoundingBox2D bbox

# 3D bounding box surrounding the object in meters
yolov8/BoundingBox3D bbox3d

# segmentation mask of the detected object
# it is only the boundary of the segmented object
yolov8/Mask mask

# keypoints for human pose estimation
yolov8/KeyPoint2DArray keypoints

# keypoints for human pose estimation
yolov8/KeyPoint3DArray keypoints3d