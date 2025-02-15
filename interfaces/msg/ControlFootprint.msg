# This msg contain predicted controls (speed, delta) and associated robot footprint polygons
std_msgs/Header header
polygon_msgs/Polygon2D[] polygons # footprints polygons collection
float32[] controls # associated sequence of controls to be resized into (-1,2) or (2,-1)
