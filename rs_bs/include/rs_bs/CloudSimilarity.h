//
// Created by rin on 06.02.21.
//

#ifndef RS_BS_CLOUDSIMILARITY_H
#define RS_BS_CLOUDSIMILARITY_H

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

bool shotSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,  pcl::PointCloud<pcl::Normal>::Ptr normals1,  pcl::PointCloud<pcl::Normal>::Ptr normals2);
bool pfhrgbSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,  pcl::PointCloud<pcl::Normal>::Ptr normals1,  pcl::PointCloud<pcl::Normal>::Ptr normals2);
bool colorSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2);
bool cloudSimilarity (PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2, pcl::PointCloud<pcl::Normal>::Ptr normals1,  pcl::PointCloud<pcl::Normal>::Ptr normals2, bool trackSimi , std::string Method) ;

void savedata(std::string methodName, float scores);

float colorPoints(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target);


void compute_descriptor_shot_color(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   typename pcl::PointCloud<pcl::SHOT1344>::Ptr &descriptors_out);

void find_feature_distance_all_points_sh(pcl::PointCloud<pcl::SHOT1344>::Ptr &source_descriptors,
                                         pcl::PointCloud<pcl::SHOT1344>::Ptr &target_descriptors,
                                         std::vector<int> &correspondences_out,
                                         std::vector<float> &correspondence_scores_out);

void find_feature_correspondence_sh(const pcl::PointCloud<pcl::SHOT1344>::Ptr &pfhs_src,
                                    const pcl::PointCloud<pcl::SHOT1344>::Ptr &pfhs_tgt,
                                    pcl::CorrespondencesPtr all_correspondences);


void compute_descriptor_pfhrgb(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                               typename pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors_out);
void find_feature_distance_all_points_pfhrgb(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &source_descriptors,
                                             pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &target_descriptors,
                                             std::vector<int> &correspondences_out,
                                             std::vector<float> &correspondence_scores_out);
void find_feature_correspondence_pfhrgb(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &pfhs_src,
                                        const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &pfhs_tgt,
                                        pcl::CorrespondencesPtr all_correspondences);


void rejectBadCorrespondences(const pcl::CorrespondencesPtr all_correspondences,
                              const pcl::PointCloud<PointT>::Ptr &src,
                              const pcl::PointCloud<PointT>::Ptr &tgt,
                              pcl::CorrespondencesPtr remaining_correspondences);

double scores(double pointCount, double pointCount2, const pcl::Correspondences all_correspondences, std::vector<int> correspondences,
            std::vector<float> correspondence_scores, const pcl::Correspondences remaining_correspondence,
            std::string method);


void regionSegmentation(pcl::PointCloud<PointT>::Ptr input, float minY, float maxY,
                        pcl::PointCloud<PointT>::Ptr output);
void compute_surface_normals(pcl::PointCloud<PointT>::Ptr &points, float normal_radius,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals_out);
void downsample(pcl::PointCloud<PointT>::Ptr &points, float leaf_size,
                pcl::PointCloud<PointT>::Ptr &downsampled_out);

void sampleRandomTransform(Eigen::Affine3f &trans, float max_angle, float max_trans);

bool colors_fit(const PointT &point_1, const PointT &point_2);

pcl::Correspondences correpfhrgb(bool ransac);

pcl::Correspondences correshot(bool ransac);

geometry_msgs::Pose transformObjectPoseInUEToCAM(geometry_msgs::Pose pose,
                                                 tf::StampedTransform &camToWorld);

void cloudToSections(pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud);
void cloudVerticalCut(pcl::PointCloud<PointT>::Ptr vertical_cut, pcl::PointCloud<PointT>::Ptr this_section1, pcl::PointCloud<PointT>::Ptr this_section2);
void cloudHorizontalCut(pcl::PointCloud<PointT>::Ptr horizontal_cut);

void find_feature_distance_all_points_sh_nndr(pcl::PointCloud<pcl::SHOT1344>::Ptr &source_descriptors,
                                              pcl::PointCloud<pcl::SHOT1344>::Ptr &target_descriptors,
                                              std::vector<int> &correspondences_out,
                                              std::vector<float> &correspondence_scores_out);
void find_feature_distance_all_points_pfhrgb_nndr(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &source_descriptors,
                                                  pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &target_descriptors,
                                                  std::vector<int> &correspondences_out,
                                                  std::vector<float> &correspondence_scores_out);

bool rsdSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
        pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2);
bool uscimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                  pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2);

#endif //RS_BS_CLOUDSIMILARITY_H
