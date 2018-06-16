#!/usr/bin/env python

# Import modules
from pcl_helper import *
import pcl

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):


    # Convert ROS msg to PCL data
    pcl_from_ros = ros_to_pcl(pcl_msg)
    
    filename = 'initial_pcl_msg.pcd'
    pcl.save(pcl_from_ros, filename)

    # Voxel Grid Downsampling
    my_voxel_grid_filter = ros_to_pcl(pcl_msg).make_voxel_grid_filter()
    
    # # Will probaly need to tweak leaf size
    LEAF_SIZE = 0.01
    my_voxel_grid_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    
    vox_filtered_cloud = my_voxel_grid_filter.filter()
    
    filename = 'voxel_filtered_cloud.pcd'
    pcl.save(vox_filtered_cloud, filename)

    #------------------------------------------------------------------------------------------------#
    # PassThrough Filter
    my_passthrough_filtered_cloud = vox_filtered_cloud.make_passthrough_filter()

    filter_axis = 'z'
    
    # Essentially this crops out anything not in the below min/max region
    axis_min = 0.6
    axis_max = 1.1
    
    my_passthrough_filtered_cloud.set_filter_field_name(filter_axis)
    my_passthrough_filtered_cloud.set_filter_limits(axis_min, axis_max)

    vox_and_passthrough_filtered_cloud = my_passthrough_filtered_cloud.filter()

    filename = 'pass_through_filtered.pcd'
    pcl.save(vox_and_passthrough_filtered_cloud, filename)

    #------------------------------------------------------------------------------------------------#
    # RANSAC Plane Segmentation
    my_seg = vox_and_passthrough_filtered_cloud.make_segmenter()

    # Segmentation models to be fitted for filtering
    my_seg.set_model_type(pcl.SACMODEL_PLANE)
    my_seg.set_model_type(pcl.SAC_RANSAC)

    # Set max distance and threshold to be considered a part of the model
    max_distance = 0.01
    my_seg.set_distance_threshold(max_distance)
    inliers, coefficient = my_seg.segment()

    #------------------------------------------------------------------------------------------------#
    # Extract inliers and outliers
    extracted_cloud_table_inliers = vox_and_passthrough_filtered_cloud.extract(inliers, negative=False)
    extracted_cloud_objects_inliers = vox_and_passthrough_filtered_cloud.extract(inliers, negative=True)
    
    table_filename = 'extracted_table_cloud.pcd'
    objects_filename = 'extracted_objects_cloud.pcd'

    pcl.save(extracted_cloud_table_inliers, table_filename)
    pcl.save(extracted_cloud_objects_inliers, objects_filename)


    # TODO: Euclidean Clustering
    # PCL Euclidiean Clustering only supports k-d trees
    # Need to remove color data
    white_cloud = XYZRGB_to_XYZ(extracted_cloud_objects_inliers)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    color_extraction_object = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    color_extraction_object.set_ClusterTolerance(0.013)
    
    # Set min/max cluster size(in points)
    # Need to tweak params
    color_extraction_object.set_MinClusterSize(150)
    color_extraction_object.set_MaxClusterSize(2600)

    # Search the kd tree for clusters
    color_extraction_object.set_SearchMethod(tree)

    # Get dem indices
    cluster_indices = color_extraction_object.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Give each cluster a color
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters but with unique colors
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    filename = 'cluster_cloud.pcd'
    pcl.save(cluster_cloud, filename)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_cloud_objects_inliers)
    ros_cloud_table = pcl_to_ros(extracted_cloud_table_inliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_color_cluster_pub.publish(ros_cluster_cloud)
    


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_color_cluster_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()