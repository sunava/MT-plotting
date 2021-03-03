#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
//RS
#include <pcl/filters/extract_indices.h>
#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/types/all_types.h>
#include <rs_bs/types/all_types.h>
#include <rs_bs/ObjectBeliefStateMatcher.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/distances.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/features/usc.h>

#include <robosherlock/CASConsumerContext.h>
#include <robosherlock/DrawingAnnotator.h>
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include <robosherlock/types/all_types.h>

#include <rs_bs/types/all_types.h>
#include <rs_bs/ObjectBeliefStateMatcher.h>

#include <rs_bs/BeliefStateCommunication.h>
#include <iostream>


#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <rs_bs/CloudSimilarity.h>


using namespace uima;

class BeliefStatePoseRefinement : public DrawingAnnotator {
private:
    typedef pcl::PointXYZRGBA PointT;
    tf::StampedTransform camToWorld;
public:

    //
    // PARAMETERS
    //
    //
    // This identifier is used to reference the CAS of another AAE that ran before
    // *this* AAE. It is accessed via rs::CASConsumerContext.
    // This Annotator will fetch an VIEW_COLOR_IMAGE from this CAS and mix it
    // with the VIEW_COLOR_IMAGE in the CAS *this* Annotator is running in.
    std::string other_cas_id_;

    pcl::PointCloud<PointT>::Ptr together_before_aligend;
    pcl::PointCloud<PointT>::Ptr together_before_aligend_scaled;
    pcl::PointCloud<PointT>::Ptr together_aligned_scaled;
    pcl::PointCloud<PointT>::Ptr together_aligned;
    pcl::PointCloud<PointT>::Ptr this_lid;

    pcl::PointCloud<PointT>::Ptr other_cas_cloud;


    double pointSize;
    bool savePictureReal;
    pcl::PointCloud<PointT>::Ptr points1_;
    pcl::PointCloud<PointT>::Ptr points2_;
    int iterationenRUN;
    std::string idO;

    geometry_msgs::Pose lidPose;
    // Check which object cluster cloud you want to see in the visualizer
    enum {
        TOGETHER_BEFORE_ALIGEND,
        TOGETHER_BEFORE_ALIGEND_SCALED,
        TOGETHER_ALIGENED_SCALED,
        TOGETHER_ALIGEND,
        THIS_LID,
        SHOTCOLOR,
    } dispMode;

    BeliefStatePoseRefinement() : DrawingAnnotator(__func__), other_cas_id_(""), pointSize(20) {

    }

    TyErrorId initialize(AnnotatorContext &ctx) {
        outInfo("initialize");
        iterationenRUN = 0;
        savePictureReal = true;
        if (ctx.isParameterDefined("otherCASId")) {
            ctx.extractValue("otherCASId", other_cas_id_);
            outInfo("Using AAE/CAS identified by '" << other_cas_id_ << "' for Clouds to refine");
        }

        dispMode = TOGETHER_BEFORE_ALIGEND;

        return UIMA_ERR_NONE;
    }

    TyErrorId destroy() {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
        outInfo("process start");
        pcl::PointCloud<PointT>::Ptr new_together_before_aligend(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr new_together_before_aligend_scaled(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr new_together_aligned_scaled(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr new_together_aligned(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr new_this_lid(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr new_other_cas_cloud(new pcl::PointCloud<PointT>());

        together_before_aligend = new_together_before_aligend;
        together_before_aligend_scaled = new_together_before_aligend_scaled;
        together_aligned_scaled = new_together_aligned_scaled;
        together_aligned = new_together_aligned;
        this_lid = new_this_lid;
        other_cas_cloud = new_other_cas_cloud;

        rs::StopWatch clock;
        rs::SceneCas this_cas(tcas);
        rs::Scene this_scene = this_cas.getScene();

        uima::CAS *other_tcas;
        other_tcas = rs::CASConsumerContext::getInstance().getCAS(other_cas_id_);

        if (!other_tcas) {
            outWarn("Couldn't fetch CAS identified by '"
                            << other_cas_id_
                            << "'. Make sure you have loaded an AAE with that name and "
                            << " that you've set 'otherCASId' in this config");
            return UIMA_ERR_ENGINE_NO_CAS;
        }

        rs::SceneCas other_cas(*other_tcas);
        rs::Scene other_scene = other_cas.getScene();

        rs_bs::ObjectBeliefStateMatcher object_bs_matcher;

        std::string object_to_refine = "PfannerGruneIcetea";
        std::string DataNames = "PfannerGruneIcetea-PfannerGruneIcetea'";

        bool tmp = 1;
        std::string name = "CloudSimiliarityLabel";
        savedata(name, tmp);

        // TODO Shouldn't this be a rs::Object ? Depends a bit on the pipeline...
        //    std::vector<rs::ObjectHypothesis> other_cas_clusters;
        //    other_scene.identifiables.filter(other_cas_clusters);
        //

        std::vector<rs::Object> other_cas_clusters;
        other_cas.get(VIEW_OBJECTS, other_cas_clusters);
        outInfo("Found " << other_cas_clusters.size() << " rs::Objects in CAS '" << other_cas_id_ << "'");

        if (other_cas_clusters.size() == 0) {
            outWarn("No Clusters found in OtherCas. Skipping ...");
            return UIMA_ERR_NONE;
        }

        int idx = -1;


        int milk_box_idx = -1;

        // Step 1:
        //   find the cluster of the object of interest
        //   Depending on the pipeline, you can either check
        //   simple annotations like color or use classifications.
        for (auto other_cas_cluster: other_cas_clusters) {
            idx++;

            std::vector<rs::Classification> classes;
            other_cas_cluster.annotations.filter(classes);

            if (classes.size() == 0) {
                outInfo("No classification information for cluster " << idx);
                continue;
            }

            std::string class_name = classes[0].classname.get();
            if (class_name == object_to_refine) {
                outInfo(object_to_refine << "found. It is at cluster idx " << idx << " with id "
                                         << other_cas_cluster.id.get());
                idO = other_cas_cluster.id.get();
                milk_box_idx = idx;
                break;
            }

        }

        if (milk_box_idx == -1) {
            outInfo("Couldn't detect milk box in other CAS. Skip this iteration.");
            return UIMA_ERR_NONE;
        }


        // Phase 2:
        //   Object has been found. Fetch pointcloud and other required data.
        rs::ObjectHypothesis &other_cas_object = other_cas_clusters[milk_box_idx];
        //object_bs_matcher.getBeliefStateObject(other_scene, other_cas_cluster);

        // Information fetched
        // Get Pointcloud and begin extraction

        pcl::PointCloud<PointT>::Ptr other_cas_cloud_ptr(new pcl::PointCloud<PointT>());
        other_cas.get(VIEW_CLOUD, *other_cas_cloud_ptr);

        if (!other_cas_object.points.has()) {
            outInfo("other_cas_object doesn't have points. Skipping.");
            return UIMA_ERR_ANNOTATOR_MISSING_INFO;
        }

        pcl::PointCloud<PointT>::Ptr other_cas_cluster_cloud(new pcl::PointCloud<PointT>());

        {
            pcl::PointIndicesPtr indices(new pcl::PointIndices());
            rs::conversion::from(static_cast<rs::ReferenceClusterPoints>(other_cas_object.points.get()).indices.get(),
                                 *indices);

            pcl::ExtractIndices<PointT> ei;
            ei.setInputCloud(other_cas_cloud_ptr);
            ei.setIndices(indices);
            ei.filter(*other_cas_cluster_cloud);
        }

        pcl::copyPointCloud(*other_cas_cluster_cloud, *together_before_aligend);

        // Step 3:
        // Lookup matching Belief State Object with PointCloud
        outInfo("filter bs clusters");
        std::vector<rs_bs::BeliefStateObject> bs_clusters;
        this_scene.identifiables.filter(bs_clusters);

//    bool bs_object_found = false;

//    rs_bs::BeliefStateObject* bs_object  = nullptr;
        outInfo("starting find_if");
        auto it = std::find_if(bs_clusters.begin(), bs_clusters.end(),
                               [&](rs_bs::BeliefStateObject &bso) {
                                   return object_bs_matcher.match(bso, other_cas_object);
                               });
        outInfo("done with find_if");

        if (it == bs_clusters.end()) {
            outError("Couldn't find Belief State object for " << object_to_refine << ". Skipping this iteration.");
            return UIMA_ERR_ANNOTATOR_MISSING_INFO;
        }

        // I think we have to work with references here as UIMA forbids the usage of pointers.
        // When trying to use ptrs on the BSObject Type, i got strange type errors.
        rs_bs::BeliefStateObject &bs_object = *it;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr this_cas_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        this_cas.get(VIEW_CLOUD, *this_cas_cloud_ptr);

        if (!bs_object.points.has()) {
            outInfo("BS Object for " << object_to_refine << " doesn't have points. Skipping.");
            return UIMA_ERR_ANNOTATOR_MISSING_INFO;
        }

        pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud(new pcl::PointCloud<PointT>());

        {
            pcl::PointIndicesPtr indices(new pcl::PointIndices());
            rs::conversion::from(static_cast<rs::ReferenceClusterPoints>(bs_object.points.get()).indices.get(),
                                 *indices);

            pcl::ExtractIndices<PointT> ei;
            ei.setInputCloud(this_cas_cloud_ptr);
            ei.setIndices(indices);
            ei.filter(*this_cas_cluster_cloud);
        }

        *together_before_aligend += *this_cas_cluster_cloud;

        // Step 4:
        // Alignment step. Use ICP for registration.

        // Match the current pose hypothesis in this_cas_cluster_cloud
        // into the other_cas_cloud as the target to calculate the error.

//    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    icp.setInputSource(this_cas_cluster_cloud);
//    icp.setInputTarget(other_cas_cluster_cloud);
//    icp.setMaximumIterations (50);
//    icp.setEuclideanFitnessEpsilon(1e-4);
//
//    pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud_aligned(new pcl::PointCloud<PointT>());
//    icp.align(*this_cas_cluster_cloud_aligned);
//
//    outInfo("ICP done. Has converged? " << icp.hasConverged()
//    << " score: " << icp.getFitnessScore());
//    outInfo("ICP found transform: " << icp.getFinalTransformation());
//
//    *dispCloud += *this_cas_cluster_cloud_aligned;
//
//    *bs_cloud         += *this_cas_cluster_cloud;
//    *bs_aligned_cloud += *this_cas_cluster_cloud_aligned;
//    *other_cas_cloud  += *other_cas_cluster_cloud;
//
//    outInfo("took: " << clock.getTime() << " ms.");




///-----------------------------------------------------------------------------------------------------------------------------
///-----------------------------------------------------------------------------------------------------------------------------

        pcl::PointCloud<PointT>::Ptr this_cloud_scaled(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr other_cloud_scaled(new pcl::PointCloud<PointT>);

        std::list<float> thisList;
        for (auto &point: *this_cas_cluster_cloud) {
            thisList.push_back(point.y);
        }

        std::list<float> otherList;
        for (auto &point: *other_cas_cluster_cloud) {
            otherList.push_back(point.y);
        }

        thisList.sort();
        otherList.sort();

        //int deleteNumber = this_cas_cluster_cloud->size()-other_cas_cluster_cloud->size();
        //int thisSectionSize = this_cas_cluster_cloud->size()-deleteNumber;
        int thisSectionSize = thisList.size() / 2;

        std::list<float> firstSection;
        std::list<float> secondSection;


        for (int i = 1; i < thisList.size(); ++i) {
            auto myList_front = thisList.begin();
            if (i <= thisSectionSize) {

                std::advance(myList_front, i);
                firstSection.push_back(*myList_front);
            }

        }

        regionSegmentation(this_cas_cluster_cloud, firstSection.front(), firstSection.back(), this_cloud_scaled);


        for (int i = 1; i < otherList.size(); ++i) {
            auto myList_front = otherList.begin();
            if (i <= thisSectionSize) {

                std::advance(myList_front, i);
                secondSection.push_back(*myList_front);
            }

        }

        regionSegmentation(other_cas_cluster_cloud, secondSection.front(), secondSection.back(), other_cloud_scaled);

///-----------------------------------------------------------------------------------------------------------------------------
///-----------------------------------------------------------------------------------------------------------------------------


        //*dispCloud += *this_cloud_scaled;

        // Step 4:
        // Alignment step. Use ICP for registration.

        // Match the current pose hypothesis in this_cas_cluster_cloud
        // into the other_cas_cloud as the target to calculate the error.



        *together_before_aligend_scaled = *other_cloud_scaled;
        *together_before_aligend_scaled += *this_cloud_scaled;

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(this_cloud_scaled);
        icp.setInputTarget(other_cloud_scaled);
        icp.setMaximumIterations (50);
        icp.setEuclideanFitnessEpsilon(1e-4);

        pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud_aligned(new pcl::PointCloud<PointT>());
        icp.align(*this_cas_cluster_cloud_aligned);

        outInfo("ICP done. Has converged? " << icp.hasConverged()
                                            << " score: " << icp.getFitnessScore());
        outInfo("ICP found transform: " << icp.getFinalTransformation());

//        GeneralizedIterativeClosestPoint6D icp;
//        icp.setInputSource(this_cloud_scaled);
//        icp.setInputTarget(other_cloud_scaled);
//        icp.setMaxCorrespondenceDistance(0.1);
//        icp.setTransformationEpsilon(1e-10);
//        icp.setEuclideanFitnessEpsilon(0.01);
//        //icp.setMaximumIterations(100);
//        icp.setRANSACIterations(12000);
//
//        pcl::PointCloud<PointT>::Ptr this_cas_cluster_cloud_aligned(new pcl::PointCloud<PointT>());
//        icp.align(*this_cas_cluster_cloud_aligned);
//
//
//        outInfo("ICP done. Has converged? " << icp.hasConverged()
//                                            << " score: " << icp.getFitnessScore());
//        outInfo("ICP found transform: " << icp.getFinalTransformation());

        pcl::PointCloud<PointT>::Ptr this_cloud_transformed(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*this_cas_cluster_cloud, *this_cloud_transformed, icp.getFinalTransformation());


        *together_aligned_scaled = *other_cloud_scaled;
        *together_aligned_scaled += *this_cas_cluster_cloud_aligned;

        *together_aligned = *other_cas_cluster_cloud;
        *together_aligned += *this_cloud_transformed;


        outInfo("took: " << clock.getTime() << " ms.");




        //pcl::io::savePCDFileASCII("output.pcd", *output);
//        pcl::io::savePCDFileASCII("this_cas_aligned.pcd", *this_cas_cluster_cloud_aligned);
//        pcl::io::savePCDFileASCII("other_cas.pcd", *other_cas_cluster_cloud);
//        pcl::io::savePCDFileASCII("merged_cloud.pcd", *other_cas_cloud);
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr this_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr other_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);


        iterationenRUN++;
        outInfo("##################################################################################################"
                        << iterationenRUN);
        outInfo("##################################################################################################"
                        << iterationenRUN);
        outInfo("##################################################################################################"
                        << iterationenRUN);




        outInfo(" together_before_align: 1, together_before_align_scaled: 2, together_align_scaled: 3, together_align: 4, this_lid_pose: 5, shot_corre: 6");

        outInfo("together_aligned size: " << together_aligned->size());
////######################################################################################################################
////                                               INIT-ALL-VARIABLES
////######################################################################################################################

        if (iterationenRUN <= 60) {
            outInfo("INIT-EVERYTHING");
            // Create some new point clouds to hold our data
            pcl::PointCloud<PointT>::Ptr points1_tmp(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr downsampled1(new pcl::PointCloud<PointT>);
            pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);

            pcl::PointCloud<PointT>::Ptr points2_tmp(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr downsampled2(new pcl::PointCloud<PointT>);
            pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);



//            float voxel_grid_leaf_size;
////            voxel_grid_leaf_size = 0.0025;
//            if (this_cloud_transformed->size() > 6000) {
//                voxel_grid_leaf_size = 0.0035;
//            } else if (this_cloud_transformed->size() > 4000) {
//                voxel_grid_leaf_size = 0.0025;
//            } else if (this_cloud_transformed->size() > 3000) {
//                voxel_grid_leaf_size = 0.0010;
//            }
            copyPointCloud(*this_cloud_transformed, *points1_tmp);
            copyPointCloud(*other_cas_cluster_cloud, *points2_tmp);
//
//
//            // Downsample the cloud
//
//
//

//            downsample(downsampled1, voxel_grid_leaf_size, points1_tmp);
//            downsample(downsampled2, voxel_grid_leaf_size, points2_tmp);
//
//
            outInfo("cloud1 size: " << downsampled1->size());
            outInfo("cloud size2: " << downsampled2->size());
            outInfo("cloud size1 downsampled: " << points1_tmp->size());
            outInfo("cloud size2 downsampled: " << points2_tmp->size());
//
//            const float feature_radius = 0.025;


////######################################################################################################################
////                                               NORMAL-ESTIMATION
////######################################################################################################################

            // Compute surface normals
            const float normal_radius = 0.008;

            pcl::PointCloud<pcl::Normal>::Ptr normals_tmp1(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::Normal>::Ptr normals_tmp2(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<PointT>::Ptr points1(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr points2(new pcl::PointCloud<PointT>);

//            normals_tmp1->is_dense = false;
//            normals1->is_dense = false;


//            compute_surface_normals(points1_tmp, normal_radius, normals_tmp1);
//            compute_surface_normals(points2_tmp, normal_radius, normals_tmp2);


            pcl::PointIndices non_NaN_ids;
            compute_normals_unOrganizedCloud(points1_tmp, normals_tmp1);
            filter_NaN_points(points1_tmp, normals_tmp1, points1, normals1, non_NaN_ids.indices);

            pcl::PointIndices non_NaN_ids2;
            compute_normals_unOrganizedCloud(points2_tmp, normals_tmp2);
            filter_NaN_points(points2_tmp, normals_tmp2, points2, normals2, non_NaN_ids2.indices);




//            pcl::copyPointCloud(*points1_tmp, indices1, *points1);
//            std::cout << "size before nan : " << normals_tmp1->points.size() << "size without nan: " << normals1->points.size () << std::endl;
//
//            std::vector<int> indices2;
//            pcl::removeNaNFromPointCloud(*normals_tmp2, *normals2, indices2);
//            pcl::copyPointCloud(*points2_tmp, indices2, *points2);
//            std::cout << "size before nan : " << normals_tmp2->points.size() << "size without nan: " << normals2->points.size () << std::endl;



////######################################################################################################################
////                                               PFH-FEATURE
////######################################################################################################################
            bool cloudSim = 0;
            bool lid = 0;
//            if (!points1->empty() && !points2->empty()) {
//
//
//                if (cloudSimilarity(points2, points1, normals2, normals1, false, "objectSimilar")) {
//                    outInfo("_______________________________Object is similar___________________________");
//                    cloudSim = 1;
//                } else {
//                    outInfo("_______________________________Object is NOT similar___________________________");
//                }
//
//            }
            points1_ = points1;
            points2_ = points2;
////######################################################################################################################
////                                               saving Pictures - FEATURE
////######################################################################################################################

            cv::Mat color, depth;
            this_cas.get(VIEW_COLOR_IMAGE_HD, color);
            std::stringstream ss_rgb;
            ss_rgb << DataNames << iterationenRUN << ".png";
            cv::imwrite(ss_rgb.str(), cv::Mat(color));


            if (savePictureReal) {
                other_cas.get(VIEW_COLOR_IMAGE_HD, color);
                std::stringstream ss_rgb_real;
                ss_rgb_real << DataNames << "_" << "real" << iterationenRUN << ".png";
                cv::imwrite(ss_rgb_real.str(), cv::Mat(color));
            }

            geometry_msgs::Pose pose;

            savePictureReal = false;


            BeliefStateCommunication::getInstance().getObjectPose(idO, pose);
//
            geometry_msgs::Pose lidP = BeliefStateCommunication::getInstance().returnSavePose();

            tf::StampedTransform camToWorld;

            camToWorld.setIdentity();
            if (other_scene.viewPoint.has()) {
                rs::conversion::from(other_scene.viewPoint.get(), camToWorld);
            } else {
                outWarn("No camera to world transformation, no further processing!");
                return UIMA_ERR_ANNOTATOR_MISSING_INFO;
            }

            geometry_msgs::Pose lidInCam = transformObjectPoseInUEToCAM(lidP, camToWorld);
            lidPose = lidInCam;

            //cloudToSections(this_cas_cluster_cloud);
            outInfo(lidInCam);

            cloudSim = 0;
            if (cloudSim  == 1) {
                pcl::KdTreeFLANN<PointT> kdtree;

                kdtree.setInputCloud(this_cloud_scaled);

                pcl::PointXYZRGBA searchPoint;

                searchPoint.x = lidInCam.position.x;
                searchPoint.y = lidInCam.position.y;
                searchPoint.z = lidInCam.position.z;

                // K nearest neighbor search

                int K = 400;

                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);

                std::cout << "K nearest neighbor search at (" << searchPoint.x
                          << " " << searchPoint.y
                          << " " << searchPoint.z
                          << ") with K=" << K << std::endl;
                pcl::PointCloud<PointT>::Ptr testing(new pcl::PointCloud<PointT>);
                if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
                        testing->push_back((*this_cloud_scaled)[pointIdxNKNSearch[i]]);
                    }

                }


                outInfo("normals");
                // Compute surface normals

                pcl::PointCloud<pcl::Normal>::Ptr normalss1(new pcl::PointCloud<pcl::Normal>);
                compute_surface_normals(testing, normal_radius, normalss1);


                kdtree.setInputCloud(other_cloud_scaled);


                std::cout << "K nearest neighbor search at (" << searchPoint.x
                          << " " << searchPoint.y
                          << " " << searchPoint.z
                          << ") with K=" << K << std::endl;
                pcl::PointCloud<PointT>::Ptr testing2(new pcl::PointCloud<PointT>);
                if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
                        testing2->push_back((*other_cloud_scaled)[pointIdxNKNSearch[i]]);
                    }

                }

                outInfo("normals");
                // Compute surface normals

                pcl::PointCloud<pcl::Normal>::Ptr normalss2(new pcl::PointCloud<pcl::Normal>);
                compute_surface_normals(testing2, normal_radius, normalss2);

                if (cloudSimilarity(testing, testing2, normalss1, normalss2, true, "lidSimilar")) {
                    outInfo("_______________________________Object is similar for new point cloud___________________________");
                    lid = 1;

                } else {
                    outInfo("_______________________________Object is NOT similar for new point cloud___________________________");
                }
            }


            *this_lid = *this_cas_cluster_cloud;

            std::string name = "LidPose";

            outInfo("took: " << clock.getTime() << " ms.");
        }
        return UIMA_ERR_NONE;
    }


    void drawImageWithLock(cv::Mat &disp) override {
        disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
    }


    void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
        const std::string &cloudname = this->name;

        if (!firstRun) {
            visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize,
                                                        cloudname);
        }
        visualizer.removeAllPointClouds();
        visualizer.removeAllShapes();

        pcl::Correspondences correspondencesp_ = correpfhrgb(false);
        pcl::Correspondences correspondencess_ = correshot(false);
        pcl::Correspondences correspondencespRANSAC_ = correpfhrgb(true);
        pcl::Correspondences correspondencessRANSAC_ = correshot(true);
        pcl::PointXYZ pp;
        pp.x = lidPose.position.x;
        pp.y = lidPose.position.y;
        pp.z = lidPose.position.z;
        visualizer.setBackgroundColor(50, 50, 50);
        switch (dispMode) {
            case TOGETHER_BEFORE_ALIGEND:
                visualizer.addPointCloud(together_before_aligend, cloudname);
                break;
            case TOGETHER_BEFORE_ALIGEND_SCALED:
                visualizer.addPointCloud(together_before_aligend_scaled, cloudname);
                break;
            case TOGETHER_ALIGENED_SCALED:
                visualizer.addPointCloud(together_aligned_scaled, cloudname);
                break;
            case TOGETHER_ALIGEND:
                visualizer.addPointCloud(together_aligned, cloudname);
                break;
            case THIS_LID:
                visualizer.addText3D("LID", pp, 0.005, 1.0, 0.4, 1.0, "1", 0);
                visualizer.addPointCloud(this_lid, cloudname);
                break;
            case SHOTCOLOR:
                corresVizuTwo(visualizer, points1_, points2_, correspondencess_, correspondencessRANSAC_);
                break;

        }
    }

    bool callbackKey(const int key, const Source source) {
        switch (key) {
            case '1':
                dispMode = TOGETHER_BEFORE_ALIGEND;
                break;
            case '2':
                dispMode = TOGETHER_BEFORE_ALIGEND_SCALED;
                break;
            case '3':
                dispMode = TOGETHER_ALIGENED_SCALED;
                break;
            case '4':
                dispMode = TOGETHER_ALIGEND;
                break;
            case '5':
                dispMode = THIS_LID;
                break;
            case '6':
                dispMode = SHOTCOLOR;
                break;
        }
        return true;
    }


    void corresVizu(pcl::visualization::PCLVisualizer &visualizer, pcl::PointCloud<pcl::Normal>::Ptr points1,
                    pcl::PointCloud<pcl::Normal>::Ptr points2) {
//        // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
//        // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
//
//        // Create some new point clouds to hold our transformed data
//        pcl::PointCloud<pcl::Normal>::Ptr points_left(new pcl::PointCloud<pcl::Normal>);
//        pcl::PointCloud<pcl::Normal>::Ptr points_right(new pcl::PointCloud<pcl::Normal>);
//
//        const Eigen::Vector3f translate(0.4, 0.0, 0.0);
//        const Eigen::Quaternionf no_rotation(0, 0, 0, 0);
//        pcl::transformPointCloud(*points2, *points_left, -translate, no_rotation);
//
//        pcl::transformPointCloud(*points1, *points_right, translate, no_rotation);
//
//        visualizer.addPointCloud(points_left, "points_left");
//        visualizer.addPointCloud(points2, "points_right");
//
//        outInfo("Done");

    }


    void corresVizuTwo(pcl::visualization::PCLVisualizer &visualizer, pcl::PointCloud<PointT>::Ptr points1,
                       pcl::PointCloud<PointT>::Ptr points2,
                       pcl::Correspondences &correspondences,
                       pcl::Correspondences &correspondences2) {
        // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
        // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

        // Create some new point clouds to hold our transformed data
        pcl::PointCloud<PointT>::Ptr points_left_l(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr points_right_l(new pcl::PointCloud<PointT>);

        // Shift the first clouds' points to the left

        const Eigen::Vector3f translate(0.8, 0.0, 0.0);
        const Eigen::Quaternionf no_rotation(0, 0, 0, 0);
        pcl::transformPointCloud(*points2, *points_left_l, -translate, no_rotation);

        const Eigen::Vector3f translate1(0.4, 0.0, 0.0);
        // Shift the second clouds' points to the right
        pcl::transformPointCloud(*points1, *points_right_l, -translate1, no_rotation);



        //visualizer.addText3D("correspondence", Pose, 0.005, 1.0, 0.4, 1.0, "1", 0);

        pcl::PointXYZ pp;
        pp.x = -6.0;
        pp.y = points_left_l.get()->points.front().y;
        pp.z = points_left_l.get()->points.front().z;
        //visualizer.addCoordinateSystem(0.5, "ref1", 0);
        //visualizer.addText("PFHRGB correspondence", -6, pp.y = points_left_l.get()->points.front().y, "ref1", 0);
        visualizer.addPointCloud(points_left_l, "points_left");
        visualizer.addPointCloud(points_right_l, "points_right");

        outInfo("the correspondence size is: " << correspondences.size());
        for (size_t i = 0; i < correspondences.size(); ++i) {
            pcl::PointXYZRGBA &src_idx = points_left_l->points[(correspondences)[i].index_query];
            pcl::PointXYZRGBA &tgt_idx = points_right_l->points[(correspondences)[i].index_match];
            std::string lineID = std::to_string(i);
            std::string lineID2 = std::to_string(i + 20000);

            // Generate a random (bright) color
            double r = (rand() % 100);
            double g = (rand() % 100);
            double b = (rand() % 100);
            double max_channel = std::max(r, std::max(g, b));
            r /= max_channel;
            g /= max_channel;
            b /= max_channel;

            visualizer.addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(src_idx, tgt_idx, r, g, b, lineID);
        }


        // Create some new point clouds to hold our transformed data
        pcl::PointCloud<PointT>::Ptr points_left_r(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr points_right_r(new pcl::PointCloud<PointT>);

        // Shift the first clouds' points to the left
        //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
        const Eigen::Vector3f translate2(0.4, 0.0, 0.0);
        pcl::transformPointCloud(*points2, *points_left_r, translate2, no_rotation);

        const Eigen::Vector3f translate3(0.8, 0.0, 0.0);
        // Shift the second clouds' points to the right
        pcl::transformPointCloud(*points1, *points_right_r, translate3, no_rotation);

        visualizer.addPointCloud(points_left_r, "points_left_r");
        visualizer.addPointCloud(points_right_r, "points_right_r");

        outInfo("the correspondence size is: " << correspondences2.size());
        for (size_t i = 0; i < correspondences2.size(); ++i) {
            pcl::PointXYZRGBA &src_idx1 = points_left_r->points[(correspondences2)[i].index_query];
            pcl::PointXYZRGBA &tgt_idx1 = points_right_r->points[(correspondences2)[i].index_match];
            std::string lineID2 = std::to_string(i + 40000);

            // Generate a random (bright) color
            double r = (rand() % 100);
            double g = (rand() % 100);
            double b = (rand() % 100);
            double max_channel = std::max(r, std::max(g, b));
            r /= max_channel;
            g /= max_channel;
            b /= max_channel;

            visualizer.addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(src_idx1, tgt_idx1, r, g, b, lineID2);

        }
        outInfo("Done");

    }



//
//    if (firstRun) {
//      visualizer.addPointCloud(dispCloud, cloudname);
//      visualizer.setPointCloudRenderingProperties(
//          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
//    } else {
//      visualizer.updatePointCloud(dispCloud, cloudname);
//      visualizer.getPointCloudRenderingProperties(
//          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
////      visualizer.removeAllShapes();
//    }


    void compute_normals_unOrganizedCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr,
                                          pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr) {
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud(cloud_ptr);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.008);
        outInfo("Normal Radius search = " << 0.008);
        ne.compute(*normals_ptr);
        outInfo("  Normal Cloud Size: " FG_BLUE << normals_ptr->points.size());
    }

    void filter_NaN_points(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &in_cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr &in_normals,
                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr &out_normals,
                           std::vector<int> &non_NaN_ids) {
        pcl::removeNaNNormalsFromPointCloud(*in_normals, *out_normals, non_NaN_ids);
        outInfo("Cloud size after filter: " << non_NaN_ids.size());
        pcl::copyPointCloud(*in_cloud, non_NaN_ids, *out_cloud);
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BeliefStatePoseRefinement)