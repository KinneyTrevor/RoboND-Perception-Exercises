# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
vox = cloud.make_voxel_grid_filter()

# Need to test different values
# Value of 1 implies one cubic meter in volume. Should start small and then go up
LEAF_SIZE = 0.01
vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)

# Call the filer function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# ------------------------------------------------

# PassThrough filter
# Create a passthrough filter object
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object
filter_axis = 'z'
passthrough
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

# Finally use the filter function to obtain the resultant point cloud
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered,filename)

# ------------------------------------------------

# RANSAC plane segmentation
# Create a segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model to be fit
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
max_distance = 0.01
seg.set_distance_threshold(max_distance)
inliers, coefficients = seg.segment()

# Extract inliers
# Negative flag basically means if you want the positive or negative of the shape extraction
extracted_inliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)

# ------------------------------------------------

# Extract outliers
# First create filter object
 outlier_filter = cloud_filtered.make_statistical_outlier_filter()

# # Set the number of neighboring points to analyze for a given point
 outlier_filter.set_mean_k(50)

# # Set threshold scale factor
 x = 1.0

# # Any point with a mean distance larer than lobal will be considered an outlier
 outlier_filter.set_std_dev_mul_thresh(x)

# # Call filter function again to apply outlier filer to pcd
 cloud_filtered = outlier_filter.filter()

# # Save pcd for tabletop object
 filename = 'extracted_outliers'
 pcl.save(extracted_inliers, filename)