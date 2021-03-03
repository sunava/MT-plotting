//
// Created by rin on 20.02.21.
//

#ifndef RS_BS_FEATURETEST_H
#define RS_BS_FEATURETEST_H

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/pfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/features/ppf.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/features/usc.h>

#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/types/all_types.h>

#include <rs_bs/types/all_types.h>
#include <rs_bs/ObjectBeliefStateMatcher.h>

#include <iostream>


#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>


using namespace pcl;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointTCloud;


void computeOutput(PointCloud<PointT>::Ptr this_cloud_filtered, PointCloud<PointT>::Ptr other_cloud_filtered);
void savedatA(std::string methodName, float scores);



#endif //RS_BS_FEATURETEST_H
