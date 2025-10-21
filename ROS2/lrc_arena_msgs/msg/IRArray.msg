# IR sensor array reading for line following
std_msgs/Header header
int32[] values        # Raw ADC values 0-1023
bool[] binary         # Thresholded line detections
float32 line_position # Computed center position [-1, 1]
