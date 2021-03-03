//
// Created by rin on 06.02.21.
//

#include "../include/rs_bs/CloudSimilarity.h"
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
#include <pcl/features/rsd.h>
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/types/all_types.h>

#include <rs_bs/types/all_types.h>
#include <rs_bs/ObjectBeliefStateMatcher.h>

#include <iostream>
#include <math.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <rs_bs/featureTest.h>

using namespace pcl;


pcl::Correspondences correspondencesp_;
pcl::Correspondences correspondencess_;
pcl::Correspondences correspondencespRANSAC_;
pcl::Correspondences correspondencessRANSAC_;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointTCloud;
std::ofstream outfile;

float test() {
    return 3.0;
}

pcl::Correspondences correpfhrgb(bool ransac) {
    if (ransac) return correspondencespRANSAC_;
    return correspondencesp_;
}

pcl::Correspondences correshot(bool ransac) {
    if (ransac)return correspondencessRANSAC_;
    return correspondencess_;
}


void savedata(std::string methodName, float scores) {
    std::string filename = methodName + ".txt";
    outfile.open(filename, std::ios_base::app); // append instead of overwrite
    outfile << std::fixed << scores << "\n";
    outfile.close();
}

float colorPoints(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target) {
    outInfo("Compute Color Points");
    double corr = 0;
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(xyz_target);

    for (std::size_t point_i = 0; point_i < xyz_source->size(); ++point_i) {
        if (!std::isfinite((*xyz_source)[point_i].x) || !std::isfinite((*xyz_source)[point_i].y) ||
            !std::isfinite((*xyz_source)[point_i].z))
            continue;

        std::vector<int> nn_indices(1);
        std::vector<float> nn_distances(1);
        if (!tree.nearestKSearch((*xyz_source)[point_i], 1, nn_indices, nn_distances))
            continue;
        std::size_t point_nn_i = nn_indices.front();

        if (colors_fit((*xyz_source)[point_i], (*xyz_target)[point_nn_i])) {
            //outInfo("true color_fit:" << corr);
            corr++;
        }
    }
    outInfo("corr colorFit: " << corr);
    outInfo("xyz size: " << xyz_source->size());
    corr = std::sqrt(corr / xyz_source->size());
    corr = std::sqrt(corr / static_cast<double> (xyz_source->size()));
    return corr;
}

bool colors_fit(const PointT &point_1, const PointT &point_2) {


    pcl::PointXYZHSV point_1_hsv;
    pcl::PointXYZHSV point_2_hsv;

    pcl::PointXYZRGBAtoXYZHSV(point_1, point_1_hsv);
    pcl::PointXYZRGBAtoXYZHSV(point_2, point_2_hsv);

    //    float hue_delta_ = 25.0f;
//    float saturation_delta_ = 45.0f;
//    float value_delta_ = 25.0f;

//    float hue_delta_ = 30.0f;
//    float saturation_delta_ = 50.0f;
//    float value_delta_ = 30.0f;

//    float hue_delta_ = 35.0f;
//    float saturation_delta_ = 55.0f;
//    float value_delta_ = 35.0f;
//
//    float hue_delta_ = 10.0f;
//    float saturation_delta_ = 20.0f;
//    float value_delta_ = 10.0f;

//worked good
    float hue_delta_ = 55.0f;
    float saturation_delta_ = 75.0f;
    float value_delta_ = 55.0f;

//    float hue_delta_ = 75.0f;
//    float saturation_delta_ = 95.0f;
//    float value_delta_ = 75.0f;



    //std::cout << "h_delta: " <<  abs(point_1_hsv.h - point_2_hsv.h) <<std::endl;
    //std::cout << "s_delta" << abs(point_1_hsv.s - point_2_hsv.s) <<std::endl;
    //std::cout << "v_delta" << abs(point_1_hsv.v- point_2_hsv.v) <<std::endl;

    float h_delta = abs(point_1_hsv.h - point_2_hsv.h);
    if (h_delta > hue_delta_) return false;
    float s_delta = abs(point_1_hsv.s - point_2_hsv.s);
    if (s_delta > saturation_delta_) return false;
    float v_delta = abs(point_1_hsv.v - point_2_hsv.v);
    return v_delta <= value_delta_;
}


void compute_descriptor_shot_color(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   typename pcl::PointCloud<pcl::SHOT1344>::Ptr &descriptors_out) {
    outInfo("Compute descriptor SHOTColor");
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    pcl::SHOTColorEstimation<PointT, pcl::Normal, pcl::SHOT1344> shot_src;
    shot_src.setSearchMethod(kdtree);
    shot_src.setInputCloud(points);
    shot_src.setInputNormals(normals);
    shot_src.setRadiusSearch(0.1);
    //0.3 best?
    //shot_src.setInputReferenceFrames('/head_mount_kinect_rgb_optical_frame');
    shot_src.compute(*descriptors_out);
}

void compute_descriptor_pfhrgb(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                               typename pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors_out) {
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    outInfo("Compute descriptor pfhrgb");
    pcl::PFHRGBEstimation<PointT, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb;
    pfhrgb.setInputCloud(points);
    pfhrgb.setInputNormals(normals);
    pfhrgb.setSearchMethod(kdtree);
    //pfhrgb.setKSearch(100);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfhrgb.setRadiusSearch(0.02);

    pfhrgb.compute(*descriptors_out);
}

void rejectBadCorrespondences(const pcl::CorrespondencesPtr all_correspondences,
                              const pcl::PointCloud<PointT>::Ptr &src,
                              const pcl::PointCloud<PointT>::Ptr &tgt,
                              pcl::CorrespondencesPtr remaining_correspondences) {

    auto RANSAC_Inlier_Threshold = 0.1;
    auto RANSAC_Iterations = 12000;




    // RandomSampleConsensus bad correspondence rejector
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> correspondence_rejector;
    correspondence_rejector.setInputSource(src);
    correspondence_rejector.setInputTarget(tgt);
    //correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
    correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
    correspondence_rejector.setRefineModel(true);//false
    correspondence_rejector.setInputCorrespondences(all_correspondences);
    correspondence_rejector.getCorrespondences(*remaining_correspondences);
}

double scores(double pointCount, double pointCount2, const pcl::Correspondences all_correspondences,
              std::vector<int> correspondences,
              std::vector<float> correspondence_scores, const pcl::Correspondences remaining_correspondence,
              std::string method) {

    //<editor-fold desc="Description">
    //    //max distance corres
//    std::string tmp;
//    float maxDis = 0.0;
//    float aver = 0.0;
//    for (int i = 0; i < all_correspondences.size(); i++) {
//        aver += all_correspondences[i].distance;
//        if (all_correspondences[i].distance > maxDis) maxDis = all_correspondences[i].distance;
//    }
//    //average distance corresr:
//    aver = aver / all_correspondences.size();
//
//
//    //max distance remaining corres
//    float maxDisRe = 0.0;
//    for (int i = 0; i < remaining_correspondence.size(); i++) {
//        if (remaining_correspondence[i].distance > maxDisRe) maxDisRe = remaining_correspondence[i].distance;
//    }
//
//    //average distance in all points but only upper half
//    float averUpper;
//    for (int i = 0; i < correspondence_scores.size(); i++) {
//        if (i >= correspondence_scores.size() / 2) {
//            averUpper += correspondence_scores[i];
//        }
//    }
//    averUpper = averUpper / (correspondence_scores.size() / 2);
//    //average distance in all points
//    std::sort(correspondence_scores.begin(), correspondence_scores.end());
//    float average = std::accumulate(correspondence_scores.begin(), correspondence_scores.end(), 0.0) /
//                    correspondence_scores.size();
//    //max distance in all points:
//    float max = correspondence_scores.back();
//
//    double sqcorr = std::sqrt(all_correspondences.size() / pointCount);
//
//    //max distance remaining corres
//    float maxWeight = 0.0;
//    float maxWeightD = 0.0;
//    for (int i = 0; i < remaining_correspondence.size(); i++) {
//        if (remaining_correspondence[i].weight > maxWeight) {
//            maxWeight = remaining_correspondence[i].weight;
//            maxWeightD = remaining_correspondence[i].distance;
//        }
//    }


//    outInfo("&&&&&&&&&&&&&&&&&");
//    outInfo("sqrt all corr: " << sqcorr);
//    outInfo("average distance in all points but only upper half: " << averUpper);
//    outInfo("average distance in all points: " << average);
//    outInfo("max distance in all points: " << max);
//    outInfo("max distance corres: " << maxDis);
//    outInfo("average distance corresr: " << aver);
//    outInfo("max distance remaining corres: " << maxDisRe);
//    outInfo("max weight D: " << maxWeightD);
//    outInfo("&&&&&&&&&&&&&&&&&");
//
//    tmp = method + "_maxWD";
//    savedata(tmp, maxWeightD);
//
//    tmp = method + "_sqrt";
//    savedata(tmp, sqcorr);
//
//    tmp = method + "_average_all";
//    savedata(tmp, average);
//
//    tmp = method + "_average_upper";
//    savedata(tmp, averUpper);
//
//    tmp = method + "_average_upper";
//    savedata(tmp, averUpper);
//
//    tmp = method + "_max_all";
//    savedata(tmp, max);
//
//    tmp = method + "_max_corr";
//    savedata(tmp, maxDis);
//
//    tmp = method + "_average_corr";
//    savedata(tmp, aver);
//
//    tmp = method + "_max_remain_corr";
//    savedata(tmp, maxDisRe);
    //</editor-fold>


    std::string tmp;


    tmp = method + "_cloud1Size";
    savedata(tmp, pointCount);

    tmp = method + "_cloud2Size";
    savedata(tmp, pointCount2);

    //max distance correspondence
    float maxDis = 0.0;
    for (int i = 0; i < all_correspondences.size(); i++) {
        if (all_correspondences[i].distance > maxDis) maxDis = all_correspondences[i].distance;
    }

    tmp = method + "_maxCor";
    savedata(tmp, maxDis);

    //max distance correspondence
    float maxDisRE = 0.0;
    for (int i = 0; i < remaining_correspondence.size(); i++) {
        if (remaining_correspondence[i].distance > maxDisRE) maxDisRE = remaining_correspondence[i].distance;
    }

    tmp = method + "_maxCorRE";
    savedata(tmp, maxDisRE);


//    //max distance correspondence
//    float maxALL = 0.0;
//    for (int i = 0; i < correspondence_scores.size(); i++) {
//        if (correspondence_scores[i]> maxDisRE) maxALL = correspondence_scores[i];
//    }
//
//    tmp = method + "_maxALL";
//    savedata(tmp, maxALL);


    double anzahlDp1 = all_correspondences.size() / pointCount;

    tmp = method + "_anzahlDp1";
    savedata(tmp, anzahlDp1);

    double anzahlDp2 = all_correspondences.size() / pointCount2;

    tmp = method + "_anzahlDp2";
    savedata(tmp, anzahlDp2);

    return anzahlDp1;
}


void find_feature_distance_all_points_sh(pcl::PointCloud<pcl::SHOT1344>::Ptr &source_descriptors,
                                         pcl::PointCloud<pcl::SHOT1344>::Ptr &target_descriptors,
                                         std::vector<int> &correspondences_out,
                                         std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::SHOT1344> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void find_feature_distance_all_points_sh_nndr(pcl::PointCloud<pcl::SHOT1344>::Ptr &source_descriptors,
                                              pcl::PointCloud<pcl::SHOT1344>::Ptr &target_descriptors,
                                              std::vector<int> &correspondences_out,
                                              std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::SHOT1344> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        if (k_squared_distances[0] / k_squared_distances[1] < 0.01) {
            correspondences_out[i] = k_indices[0];
            correspondence_scores_out[i] = k_squared_distances[0];
        }
    }
}


void find_feature_correspondence_sh(const pcl::PointCloud<pcl::SHOT1344>::Ptr &pfhs_src,
                                    const pcl::PointCloud<pcl::SHOT1344>::Ptr &pfhs_tgt,
                                    pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
    est.setInputSource(pfhs_src);
    est.setInputTarget(pfhs_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}


void find_feature_distance_all_points_pfhrgb(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &source_descriptors,
                                             pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &target_descriptors,
                                             std::vector<int> &correspondences_out,
                                             std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHRGBSignature250> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void find_feature_distance_all_points_pfhrgb_nndr(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &source_descriptors,
                                                  pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &target_descriptors,
                                                  std::vector<int> &correspondences_out,
                                                  std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHRGBSignature250> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        if (k_squared_distances[0] / k_squared_distances[1] < 0.01) {
            correspondences_out[i] = k_indices[0];
            correspondence_scores_out[i] = k_squared_distances[0];
        }
    }
}

void find_feature_correspondence_pfhrgb(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &pfhs_src,
                                        const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &pfhs_tgt,
                                        pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
    est.setInputSource(pfhs_src);
    est.setInputTarget(pfhs_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}


void compute_surface_normals(pcl::PointCloud<PointT>::Ptr &points, float normal_radius,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals_out) {
    pcl::NormalEstimation<PointT, pcl::Normal> norm_est;

    // Use a FLANN-based KdTree to perform neighborhood searches
    norm_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));

    // Specify the size of the local neighborhood to use when computing the surface normals
    norm_est.setRadiusSearch(normal_radius);

    // Set the input points
    norm_est.setInputCloud(points);

    // Estimate the surface normals and store the result in "normals_out"
    norm_est.compute(*normals_out);

}


void downsample(pcl::PointCloud<PointT>::Ptr &points, float leaf_size,
                pcl::PointCloud<PointT>::Ptr &downsampled_out) {
    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud(points);
    vox_grid.filter(*downsampled_out);
}

void sampleRandomTransform(Eigen::Affine3f &trans, float max_angle, float max_trans) {
    srand(0);
    // Sample random transform
    Eigen::Vector3f axis((float) rand() / RAND_MAX, (float) rand() / RAND_MAX, (float) rand() / RAND_MAX);
    axis.normalize();
    float angle = (float) rand() / RAND_MAX * max_angle;
    Eigen::Vector3f translation((float) rand() / RAND_MAX, (float) rand() / RAND_MAX, (float) rand() / RAND_MAX);
    translation *= max_trans;
    Eigen::Affine3f rotation(Eigen::AngleAxis<float>(angle, axis));
    trans = Eigen::Translation3f(translation) * rotation;
}


void regionSegmentation(pcl::PointCloud<PointT>::Ptr input, float minY, float maxY,
                        pcl::PointCloud<PointT>::Ptr output) {
    pcl::PassThrough<PointXYZRGBA> ptfilter2(
            true); // Initializing with true will allow us to extract the removed indices
    ptfilter2.setInputCloud(input);
    ptfilter2.setFilterFieldName("y");
    ptfilter2.setFilterLimits(minY, maxY);
    ptfilter2.filter(*output);

}


bool shotSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                    pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2) {
    outInfo("Compute SHOT features");
    // Compute PFH features
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors1_SH(new pcl::PointCloud<pcl::SHOT1344>);
    compute_descriptor_shot_color(points1, normals1, descriptors1_SH);
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors2_SH(new pcl::PointCloud<pcl::SHOT1344>);
    compute_descriptor_shot_color(points2, normals2, descriptors2_SH);


    outInfo("Find feature correspondences");

    pcl::CorrespondencesPtr all_correspondences_SH(new pcl::Correspondences());
    find_feature_correspondence_sh(descriptors1_SH, descriptors2_SH, all_correspondences_SH);



    // Find feature correspondences
    std::vector<int> correspondences_SH;
    std::vector<float> correspondence_scores_SH;
//    find_feature_distance_all_points_sh(descriptors1_SH, descriptors2_SH, correspondences_SH,
//                                        correspondence_scores_SH);


    outInfo("rejectBadCorrespondence");
    pcl::CorrespondencesPtr remaining_correspondences_SH(new pcl::Correspondences());
    rejectBadCorrespondences(all_correspondences_SH, points1, points2, remaining_correspondences_SH);


    auto sqcorr = scores(points1->size(), points2->size(), *all_correspondences_SH, correspondences_SH,
                         correspondence_scores_SH,
                         *remaining_correspondences_SH,
                         "ShotColor");
    outInfo("_______________________________shot all corres: " << all_correspondences_SH->size());
    outInfo("_______________________________shot non rejected corres: " << remaining_correspondences_SH->size());
    outInfo("_______________________________shot sqcorr: " << sqcorr);


    correspondencess_ = *all_correspondences_SH;
    correspondencessRANSAC_ = *remaining_correspondences_SH;

    // Find feature correspondences
    std::vector<int> correspondences_SH_nndr;
    std::vector<float> correspondence_scores_SH_nndr;
//    find_feature_distance_all_points_sh_nndr(descriptors1_SH, descriptors2_SH, correspondences_SH_nndr,
//                                        correspondence_scores_SH_nndr);
//
//    std::string name = "shot_nndr";
//    savedata(name, correspondence_scores_SH_nndr.size());

    return sqcorr >= 0.08;
}


bool pfhrgbSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                      pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2) {
    outInfo("Compute pfhrgb features");
    // Compute PFH features
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors1_pfhrgb(
            new pcl::PointCloud<pcl::PFHRGBSignature250>);
    compute_descriptor_pfhrgb(points1, normals1, descriptors1_pfhrgb);
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors2_pfhrgb(
            new pcl::PointCloud<pcl::PFHRGBSignature250>);
    compute_descriptor_pfhrgb(points2, normals2, descriptors2_pfhrgb);


    outInfo("Find feature correspondences");

    pcl::CorrespondencesPtr all_correspondences_pfhrgb(new pcl::Correspondences());
    find_feature_correspondence_pfhrgb(descriptors1_pfhrgb, descriptors2_pfhrgb,
                                       all_correspondences_pfhrgb);
    // Find feature correspondences
    std::vector<int> correspondences_pfhrgb;
    std::vector<float> correspondence_scores_pfhrgb;
    find_feature_distance_all_points_pfhrgb(descriptors1_pfhrgb, descriptors2_pfhrgb,
                                            correspondences_pfhrgb,
                                            correspondence_scores_pfhrgb);
    outInfo("all correspondence size: " << all_correspondences_pfhrgb->size());


    outInfo("rejectBadCorrespondence");
    pcl::CorrespondencesPtr remaining_correspondences_pfhrgb(new pcl::Correspondences());
    rejectBadCorrespondences(all_correspondences_pfhrgb, points1, points2,
                             remaining_correspondences_pfhrgb);

    // Visualize the two point clouds and their feature correspondences
    //visualize_correspondences(points1, points2, *remaining_correspondences_pfhrgb);


    auto sqcorr = scores(points1->size(), points2->size(), *all_correspondences_pfhrgb, correspondences_pfhrgb,
                         correspondence_scores_pfhrgb,
                         *remaining_correspondences_pfhrgb, "PFHRGB");
    outInfo("_______________________________pfhrgb all corres: " << all_correspondences_pfhrgb);
    outInfo("_______________________________pfhrgb all corres: " << remaining_correspondences_pfhrgb);
    outInfo("_______________________________pfhrgb sqcorr: " << sqcorr);

    correspondencesp_ = *all_correspondences_pfhrgb;
    correspondencespRANSAC_ = *remaining_correspondences_pfhrgb;

    // Find feature correspondences
    std::vector<int> correspondences_pfhrgb_nndr;
    std::vector<float> correspondence_scores_pfhrgb_nndr;
    find_feature_distance_all_points_pfhrgb_nndr(descriptors1_pfhrgb, descriptors2_pfhrgb,
                                                 correspondences_pfhrgb_nndr,
                                                 correspondence_scores_pfhrgb_nndr);

    std::string name = "pfhrgb_nndr";
    savedata(name, correspondence_scores_pfhrgb_nndr.size());
    return sqcorr >= 0.014;
}

bool colorSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2) {
    float tmp = 0.0;
    tmp = colorPoints(points1, points2);
    std::string name = "ColorFit_all";
    savedata(name, tmp);
    outInfo("_______________________________colorNN sqcorr: " << tmp);
    return tmp >= 0.0210;
}


bool cloudSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                     pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2,
                     bool trackSimi, std::string Method) {

    computeOutput(points1, points2); //hausdorf, rift, DataIndex, DataNN, DataPlane
    rsdSimilarity(points1, points2, normals1, normals2);
    //uscimilarity(points1, points2, normals1, normals2);
    int vote = 0;
    if (shotSimilarity(points1, points2, normals1, normals2)) {
        outInfo("_______________________________SHOT voted for similar Object___________________________");
        vote++;
    }
//    if (pfhrgbSimilarity(points1, points2, normals1, normals2)) {
//        outInfo("_______________________________PFHRGB voted for similar Object___________________________");
//        vote++;
//    }
//    if (colorSimilarity(points1, points2)) {
//        outInfo("_______________________________ColorNN voted for similar Object___________________________");
//        vote++;
//    }

    if (trackSimi) {
        bool tmp = vote >= 1;
        std::string name = "CloudSimiliarityResult";
        name += Method;
        savedata(name, tmp);
    }
    return vote >= 1;
}

geometry_msgs::Pose transformObjectPoseInUEToCAM(geometry_msgs::Pose pose,
                                                 tf::StampedTransform &camToWorld) {

    tf::Transform worldToUE4;
    tf::Vector3 worldToUE4_loc = tf::Vector3(0.892, -1.75, 0);
    tf::Quaternion worldToUE4_rot = tf::Quaternion(0, 0, 1, 0);
    worldToUE4.setOrigin(worldToUE4_loc);
    worldToUE4.setRotation(worldToUE4_rot);


    geometry_msgs::Pose p, q;
    p = pose;
//    p = convertFromRSCameraPose(pose);

    tf::Transform object_pose;
    tf::Vector3 object_pose_loc(p.position.x, p.position.y, p.position.z);
    tf::Quaternion object_pose_rot(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    object_pose.setOrigin(object_pose_loc);
    object_pose.setRotation(object_pose_rot);

    tf::Transform object_transform_in_ue4 = camToWorld.inverse() * worldToUE4.inverse() * object_pose;

    q.position.x = object_transform_in_ue4.getOrigin().getX();
    q.position.y = object_transform_in_ue4.getOrigin().getY();
    q.position.z = object_transform_in_ue4.getOrigin().getZ();

    q.orientation.x = object_transform_in_ue4.getRotation().getX();
    q.orientation.y = object_transform_in_ue4.getRotation().getY();
    q.orientation.z = object_transform_in_ue4.getRotation().getZ();
    q.orientation.w = object_transform_in_ue4.getRotation().getW();


    tf::Quaternion r(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w);

    geometry_msgs::Pose result;
    result.position.x = q.position.x;
    result.position.y = q.position.y;
    result.position.z = q.position.z;
    result.orientation.x = r.getX();
    result.orientation.y = r.getY();
    result.orientation.z = r.getZ();
    result.orientation.w = r.getW();

    return result;
}


void cloudHorizontalCut(pcl::PointCloud<PointT>::Ptr horizontal_cut) {
    pcl::PointCloud<PointT>::Ptr this_cloud_scaled(new pcl::PointCloud<PointT>);

    std::list<float> thisList;
    for (auto &point: *horizontal_cut) {
        thisList.push_back(point.y);
    }

    thisList.sort();

    int thisSectionSize = thisList.size() / 3;

    std::list<float> firstSection;
    std::list<float> secondSection;


    for (int i = 1; i < thisList.size(); ++i) {
        auto myList_front = thisList.begin();
        if (i <= thisSectionSize) {

            std::advance(myList_front, i);
            firstSection.push_back(*myList_front);
        }

    }

    regionSegmentation(horizontal_cut, firstSection.front(), firstSection.back(), horizontal_cut);

}

void cloudVerticalCut(pcl::PointCloud<PointT>::Ptr vertical_cut, pcl::PointCloud<PointT>::Ptr this_section1,
                      pcl::PointCloud<PointT>::Ptr this_section2) {

    std::list<float> thisList;
    for (auto &point: *vertical_cut) {
        thisList.push_back(point.z);
    }

    thisList.sort();

    int thisSectionSize = thisList.size() / 3;

    std::list<float> firstSection;
    std::list<float> secondSection;


    for (int i = 1; i < thisList.size(); ++i) {
        auto myList_front = thisList.begin();
        if (i <= thisSectionSize) {

            std::advance(myList_front, i);
            firstSection.push_back(*myList_front);
        } else if (i <= thisSectionSize * 2) {
            std::advance(myList_front, i);
            secondSection.push_back(*myList_front);
        }

    }

    regionSegmentation(vertical_cut, firstSection.front(), firstSection.back(), this_section1);
    regionSegmentation(vertical_cut, secondSection.front(), secondSection.back(), this_section2);
}

void cloudToSections(pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud) {
    pcl::PointCloud<PointT>::Ptr this_cloud_scaled(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*this_cas_cluster_cloud, *this_cloud_scaled);
    pcl::PointCloud<PointT>::Ptr this_section1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr this_section2(new pcl::PointCloud<PointT>);

    cloudHorizontalCut(this_cloud_scaled);
    cloudVerticalCut(this_cloud_scaled, this_section1, this_section2);

    pcl::io::savePCDFileASCII("this_section1.pcd", *this_section1);
    pcl::io::savePCDFileASCII("this_section2.pcd", *this_section2);
}


std::vector<float> readDataBase(std::string stringPath, std::vector<float> method) {

    std::fstream myFile(stringPath, std::ios_base::in);
    float a;
    while (myFile >> a) {
        method.push_back(a);
    }
    outInfo(method.front());
    return method;

}


void find_feature_correspondence_shape_context(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &pfhs_src,
                                               const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &pfhs_tgt,
                                               pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::ShapeContext1980, pcl::ShapeContext1980> est;
    est.setInputSource(pfhs_src);
    est.setInputTarget(pfhs_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}


void find_feature_distance_all_points_usc(pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &source_descriptors,
                                          pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &target_descriptors,
                                          std::vector<int> &correspondences_out,
                                          std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::UniqueShapeContext1960> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void find_feature_correspondence_usc(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &pfhs_src,
                                     const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &pfhs_tgt,
                                     pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::UniqueShapeContext1960, pcl::UniqueShapeContext1960> est;
    est.setInputSource(pfhs_src);
    est.setInputTarget(pfhs_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}


void find_feature_distance_all_points_rsd(pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &source_descriptors,
                                          pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &target_descriptors,
                                          std::vector<int> &correspondences_out,
                                          std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PrincipalRadiiRSD> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void find_feature_correspondence_rsd(const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &rsd_src,
                                     const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &rsd_tgt,
                                     pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD, pcl::PrincipalRadiiRSD> est;
    est.setInputSource(rsd_src);
    est.setInputTarget(rsd_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}

void compute_descriptor_pfh(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            float feature_radius,
                            typename pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out) {
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    outInfo("Compute descriptor PFH");
    // Create a PFHEstimation object
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod(kdtree);

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch(feature_radius);

    // Set the input points and surface normals
    pfh_est.setInputCloud(points);
    pfh_est.setInputNormals(normals);

    // Compute the features
    pfh_est.compute(*descriptors_out);
}

void find_feature_distance_all_points_shape_context(pcl::PointCloud<pcl::ShapeContext1980>::Ptr &source_descriptors,
                                                    pcl::PointCloud<pcl::ShapeContext1980>::Ptr &target_descriptors,
                                                    std::vector<int> &correspondences_out,
                                                    std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::ShapeContext1980> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void compute_descriptor_shape_context(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                      float feature_radius,
                                      typename pcl::PointCloud<pcl::ShapeContext1980>::Ptr &descriptors_out) {
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    outInfo("Compute descriptor 3DSC");
    // 3DSC estimation object.
    pcl::ShapeContext3DEstimation<PointT, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setInputCloud(points);
    sc3d.setInputNormals(normals);
    sc3d.setSearchMethod(kdtree);
    // The minimal radius is generally set to approx. 1/10 of the search radius, while the pt. density radius is generally set to 1/5
    sc3d.setRadiusSearch(0.1);
    sc3d.setPointDensityRadius(0.04);
    sc3d.setMinimalRadius(0.02);


    sc3d.compute(*descriptors_out);
}


void compute_descriptor_usc(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            typename pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &descriptors_out) {
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    outInfo("Compute descriptor USC");
    // USC estimation object.
    pcl::UniqueShapeContext<PointT, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
    usc.setInputCloud(points);
    // Search radius, to look for neighbors. It will also be the radius of the support sphere.
    usc.setRadiusSearch(0.1);

    usc.compute(*descriptors_out);
}


void find_feature_distance_all_points(pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                                      std::vector<int> &correspondences_out,
                                      std::vector<float> &correspondence_scores_out) {
    // Resize the output vector
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (size_t i = 0; i < source_descriptors->size(); ++i) {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void compute_descriptor_rsd(pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            typename pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr &descriptors_out) {
    outInfo("Compute descriptor rsd");
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    // RSD estimation object.
    pcl::RSDEstimation<PointT, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
    rsd.setInputCloud(points);
    rsd.setInputNormals(normals);
    rsd.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    rsd.setRadiusSearch(0.1);
    // Plane radius. Any radius larger than this is considered infinite (a plane).
    // rsd.setPlaneRadius(0.3);
    // Do we want to save the full distance-angle histograms?
    rsd.setSaveHistograms(false);
    rsd.compute(*descriptors_out);
}

void find_feature_correspondence(const pcl::PointCloud<pcl::PFHSignature125>::Ptr &pfhs_src,
                                 const pcl::PointCloud<pcl::PFHSignature125>::Ptr &pfhs_tgt,
                                 pcl::CorrespondencesPtr all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    est.setInputSource(pfhs_src);
    est.setInputTarget(pfhs_tgt);
    est.determineReciprocalCorrespondences(*all_correspondences);
}


bool rsdSimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                    pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2) {
    outInfo("Compute SHOT features");
    // Compute PFH features
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors1_SH(new pcl::PointCloud<pcl::PrincipalRadiiRSD>);
    compute_descriptor_rsd(points1, normals1, descriptors1_SH);
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors2_SH(new pcl::PointCloud<pcl::PrincipalRadiiRSD>);
    compute_descriptor_rsd(points2, normals2, descriptors2_SH);


    outInfo("Find feature correspondences");

    pcl::CorrespondencesPtr all_correspondences_SH(new pcl::Correspondences());
    find_feature_correspondence_rsd(descriptors1_SH, descriptors2_SH, all_correspondences_SH);


    std::vector<int> correspondences_SH;
    std::vector<float> correspondence_scores_SH;
//    find_feature_distance_all_points_sh(descriptors1_SH, descriptors2_SH, correspondences_SH,
//                                        correspondence_scores_SH);


    outInfo("rejectBadCorrespondence");
    pcl::CorrespondencesPtr remaining_correspondences_SH(new pcl::Correspondences());
    rejectBadCorrespondences(all_correspondences_SH, points1, points2, remaining_correspondences_SH);


    auto sqcorr = scores(points1->size(), points2->size(), *all_correspondences_SH, correspondences_SH,
                         correspondence_scores_SH,
                         *remaining_correspondences_SH,
                         "RSD");

    correspondencess_ = *all_correspondences_SH;
    correspondencessRANSAC_ = *remaining_correspondences_SH;

    // Find feature correspondences
    std::vector<int> correspondences_SH_nndr;
    std::vector<float> correspondence_scores_SH_nndr;
//    find_feature_distance_all_points_sh_nndr(descriptors1_SH, descriptors2_SH, correspondences_SH_nndr,
//                                        correspondence_scores_SH_nndr);
//
//    std::string name = "shot_nndr";
//    savedata(name, correspondence_scores_SH_nndr.size());

    return sqcorr >= 0.365;
}

bool uscimilarity(PointCloud<PointT>::Ptr points1, PointCloud<PointT>::Ptr points2,
                   pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2) {
    outInfo("Compute SHOT features");
    // Compute PFH features
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descriptors1_SH(new pcl::PointCloud<pcl::UniqueShapeContext1960>);
    compute_descriptor_usc(points1, normals1, descriptors1_SH);
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descriptors2_SH(new pcl::PointCloud<pcl::UniqueShapeContext1960>);
    compute_descriptor_usc(points2, normals2, descriptors2_SH);


    outInfo("Find feature correspondences");

    pcl::CorrespondencesPtr all_correspondences_SH(new pcl::Correspondences());
    find_feature_correspondence_usc(descriptors1_SH, descriptors2_SH, all_correspondences_SH);


    std::vector<int> correspondences_SH;
    std::vector<float> correspondence_scores_SH;
//    find_feature_distance_all_points_sh(descriptors1_SH, descriptors2_SH, correspondences_SH,
//                                        correspondence_scores_SH);


    outInfo("rejectBadCorrespondence");
    pcl::CorrespondencesPtr remaining_correspondences_SH(new pcl::Correspondences());
    rejectBadCorrespondences(all_correspondences_SH, points1, points2, remaining_correspondences_SH);


    auto sqcorr = scores(points1->size(), points2->size(), *all_correspondences_SH, correspondences_SH,
                         correspondence_scores_SH,
                         *remaining_correspondences_SH,
                         "USC");

    correspondencess_ = *all_correspondences_SH;
    correspondencessRANSAC_ = *remaining_correspondences_SH;

    // Find feature correspondences
    std::vector<int> correspondences_SH_nndr;
    std::vector<float> correspondence_scores_SH_nndr;
//    find_feature_distance_all_points_sh_nndr(descriptors1_SH, descriptors2_SH, correspondences_SH_nndr,
//                                        correspondence_scores_SH_nndr);
//
//    std::string name = "shot_nndr";
//    savedata(name, correspondence_scores_SH_nndr.size());

    return sqcorr >= 0.365;
}
