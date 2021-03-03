#include "../include/rs_bs/featureTest.h"
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
#include <pcl/features/intensity_gradient.h>
#include <rs_bs/types/all_types.h>
#include <rs_bs/ObjectBeliefStateMatcher.h>
#include <pcl/features/rift.h>

#include <iostream>
#include <math.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>


using namespace pcl;
typedef pcl::PointXYZRGBA PointT;
std::ofstream outfile;


float hausDorfCompute(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target) {
    outInfo("Computing Hausdorfdistance");


    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud(xyz_target);
    float max_dist_a = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < xyz_source->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_b.nearestKSearch(xyz_source->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
            max_dist_a = sqr_distances[0];
    }

    // compare B to A
    pcl::search::KdTree<PointT> tree_a;
    tree_a.setInputCloud(xyz_source);
    float max_dist_b = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < xyz_target->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_a.nearestKSearch(xyz_target->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
            max_dist_b = sqr_distances[0];
    }

    max_dist_a = std::sqrt(max_dist_a);
    max_dist_b = std::sqrt(max_dist_b);

    float dist_hausdorf1 = std::max(max_dist_a, max_dist_b);
    return dist_hausdorf1;
}

float indexRMSE(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target) {
    outInfo("Computing using the equal indices correspondence heuristic.");

    float rmse = 0.0f;


    for (std::size_t point_i = 0; point_i < xyz_source->size(); ++point_i) {
        if (!std::isfinite((*xyz_source)[point_i].x) || !std::isfinite((*xyz_source)[point_i].y) ||
            !std::isfinite((*xyz_source)[point_i].z))
            continue;
        if (!std::isfinite((*xyz_target)[point_i].x) || !std::isfinite((*xyz_target)[point_i].y) ||
            !std::isfinite((*xyz_target)[point_i].z))
            continue;


        float dist = squaredEuclideanDistance((*xyz_source)[point_i], (*xyz_target)[point_i]);
        rmse += dist;

    }
    rmse = std::sqrt(rmse / static_cast<float> (xyz_source->size()));


    outInfo("RMSE Error index: " << rmse);

    return rmse;
}

float nnRMSE(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target) {
    outInfo ("Computing using the nearest neighbor correspondence heuristic.\n");

    double rmse = 0.0f;

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(xyz_target);

    for (std::size_t point_i = 0; point_i < xyz_source->size(); ++point_i) {
        if (!std::isfinite((*xyz_source)[point_i].x) || !std::isfinite((*xyz_source)[point_i].y) ||
            !std::isfinite((*xyz_source)[point_i].z))
            continue;

        std::vector<int> nn_indices(1);
        std::vector<float> nn_distances(1);
        if (!tree.nearestKSearch((*xyz_source)[point_i], 3, nn_indices, nn_distances))
            continue;
        std::size_t point_nn_i = nn_indices.front();

        double dist = squaredEuclideanDistance((*xyz_source)[point_i], (*xyz_target)[point_nn_i]);
        rmse += dist;

    }
    rmse = std::sqrt(rmse / static_cast<double> (xyz_source->size()));
    outInfo("RMSE Error nn: " << rmse);

    double rmse_nn = rmse;
    return rmse_nn;

}

float nnPlaneRMSE(PointCloud<PointT>::Ptr xyz_source, PointCloud<PointT>::Ptr xyz_target) {
    outInfo("Computing using the nearest neighbor plane projection correspondence heuristic.\n");

    double rmse = 0.0f;


    pcl::PointCloud<pcl::Normal>::Ptr cloud_n(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> nel;
    ne.setInputCloud(xyz_source);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.25);
    outInfo("Normal Radius search = " << 0.25);
    ne.compute(*cloud_n);
    outInfo("  Normal Cloud Size: " FG_BLUE << cloud_n->points.size());


    KdTreeFLANN<PointT>::Ptr treeb(new KdTreeFLANN<PointT>());
    treeb->setInputCloud(xyz_target);

    for (std::size_t point_i = 0; point_i < xyz_source->size(); ++point_i) {
        if (!std::isfinite((*xyz_source)[point_i].x) || !std::isfinite((*xyz_source)[point_i].y) ||
            !std::isfinite((*xyz_source)[point_i].z))
            continue;

        std::vector<int> nn_indices(1);
        std::vector<float> nn_distances(1);
        if (!treeb->nearestKSearch((*xyz_source)[point_i], 1, nn_indices, nn_distances))
            continue;
        std::size_t point_nn_i = nn_indices.front();

        Eigen::Vector3f normal_target = (*cloud_n)[point_nn_i].getNormalVector3fMap(),
                point_source = (*xyz_source)[point_i].getVector3fMap(),
                point_target = (*xyz_target)[point_nn_i].getVector3fMap();

        float dist = normal_target.dot(point_source - point_target);
        rmse += dist * dist;


    }
    rmse = std::sqrt(rmse / static_cast<float> (xyz_source->size()));
    return rmse;
}


float matchRIFTFeaturesKnn(pcl::PointCloud<pcl::Histogram<32>>::Ptr descriptors1,
                           pcl::PointCloud<pcl::Histogram<32>>::Ptr descriptors2) {


    for (size_t i = 0; i < descriptors1->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        for (float elem: descriptors1->points[i].histogram) {
            if (isinf(elem) || isnan(elem)) {
                outInfo("ergebniss isfinite: " << isfinite(elem));
                outInfo("ergebniss nan: " << isnan(elem));
                outInfo("matchRift: " << elem);
                return NAN;
            }
        }
    }

    for (size_t i = 0; i < descriptors2->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        for (float elem: descriptors2->points[i].histogram) {
            if (isinf(elem) || isnan(elem)) {
                outInfo("ergebniss isfinite: " << isfinite(elem));
                outInfo("ergebniss nan: " << isnan(elem));
                outInfo("matchRift: " << elem);
                return NAN;
            }
        }
    }



    // compare A to B
    pcl::KdTreeFLANN<pcl::Histogram<32>> tree_b;
    tree_b.setInputCloud(descriptors2);
    float max_dist_a = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < descriptors1->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_b.nearestKSearch(descriptors1->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
            max_dist_a = sqr_distances[0];
    }

    // compare B to A
    pcl::KdTreeFLANN<pcl::Histogram<32>> tree_a;
    tree_a.setInputCloud(descriptors1);
    float max_dist_b = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < descriptors2->points.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_a.nearestKSearch(descriptors2->points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
            max_dist_b = sqr_distances[0];
    }

    max_dist_a = std::sqrt(max_dist_a);
    max_dist_b = std::sqrt(max_dist_b);

    float dist_hausdorf1 = std::max(max_dist_a, max_dist_b);
    return dist_hausdorf1;

}


float matchRSD(pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors1,
               pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors2) {
    pcl::registration::CorrespondenceEstimation<pcl::PrincipalRadiiRSD, pcl::PrincipalRadiiRSD> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(descriptors1);
    est.setInputTarget(descriptors2);
    est.determineCorrespondences(*correspondences);

    return correspondences.get()->size();


}


float matchPFH(pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1,
               pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2) {
    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(descriptors1);
    est.setInputTarget(descriptors2);
    est.determineCorrespondences(*correspondences);

    return correspondences.get()->size();
}


pcl::PointCloud<pcl::Histogram<32>>::Ptr riftCompute(PointCloud<PointT>::Ptr cloud_filtered) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Histogram<32>>::Ptr descriptors(new pcl::PointCloud<pcl::Histogram<32>>());

    pcl::copyPointCloud(*cloud_filtered, *cloudColor);


    // Convert the RGB to intensity.
    pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);


    outInfo("Compute Normals");
    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloudIntensity);
    normalEstimation.setRadiusSearch(0.008);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    outInfo("Intensity gradients");
    // Compute the intensity gradients.
    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient,
            pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
    ge.setInputCloud(cloudIntensity);
    ge.setInputNormals(normals);
    ge.setRadiusSearch(0.03);
    ge.compute(*gradients);


    outInfo("RIFTESTIMATION");
    // RIFT estimation object.
    //pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32> rift;
    pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32>> rift;
    outInfo("setting everything");
    rift.setInputCloud(cloudIntensity);
    rift.setSearchMethod(kdtree);
    // Set the intensity gradients to use.
    rift.setInputGradient(gradients);
    // Radius, to get all neighbors within.
    rift.setRadiusSearch(10.0);
    // Set the number of bins to use in the distance dimension.
    rift.setNrDistanceBins(4);
    // Set the number of bins to use in the gradient orientation dimension.
    rift.setNrGradientBins(8);
    // Note: you must change the output histogram size to reflect the previous values.


    outInfo("compute");
    rift.compute(*descriptors);
    return descriptors;
}




void savedatA(std::string methodName, float scores) {
    std::string filename = methodName + ".txt";
    outfile.open(filename, std::ios_base::app); // append instead of overwrite
    outfile << std::fixed << scores << "\n";
    outfile.close();
}
void computeOutput(PointCloud<PointT>::Ptr this_cloud_filtered, PointCloud<PointT>::Ptr other_cloud_filtered) {
    float tmp;
    std::string DataNames = "test";
    int iterationenRUN = 0;

    tmp = indexRMSE(this_cloud_filtered, other_cloud_filtered);
    if (!std::isinf(tmp) && !std::isnan(tmp)) {
        savedatA("DataIndex", tmp);
    }

    tmp = nnRMSE(this_cloud_filtered, other_cloud_filtered);
    if (!std::isinf(tmp) && !std::isnan(tmp)) {
        savedatA("DataNN", tmp);
    }

    tmp = nnPlaneRMSE(this_cloud_filtered, other_cloud_filtered);
    if (!std::isinf(tmp) && !std::isnan(tmp)) {
        savedatA("DataNNPlane", tmp);
    }

    tmp = matchRIFTFeaturesKnn(riftCompute(this_cloud_filtered), riftCompute(other_cloud_filtered));
    if (!std::isinf(tmp) && !std::isnan(tmp)) {
        savedatA("DataRift", tmp);
    }


    tmp = hausDorfCompute(this_cloud_filtered, other_cloud_filtered);
    if (!std::isinf(tmp) && !std::isnan(tmp)) {
        savedatA("DataHausdorf", tmp);
    }
//
//    tmp = matchPFH(pfhCompute(this_cloud_filtered), pfhCompute(other_cloud_filtered));
//    if (!std::isinf(tmp) && !std::isnan(tmp)) {
//        savedatA("DataPFH", tmp);
//    }


}
