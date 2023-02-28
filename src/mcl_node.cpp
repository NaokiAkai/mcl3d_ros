/****************************************************************************
 * mcl3d_ros: 3D Monte Carlo localization for ROS use
 * Copyright (C) 2023 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <mcl3d_ros/MCL.h>
#include <mcl3d_ros/IMU.h>
#include <chrono>

class MCLNode {
private:
    ros::NodeHandle nh_;

    std::string mapYamlFile_;

    ros::Subscriber sensorPointsSub_, imuSub_, odomSub_, initialPoseSub_;

    std::string mapFrame_, odomFrame_, baseLinkFrame_, laserFrame_, optPoseFrame_;

    ros::Publisher particlesPub_, optParticlesPub_, posePub_, optPosePub_, alignedPointsOptPub_, mapPointsPub_;

    mcl3d::MCL mcl_;
    mcl3d::Pose initialPose_, initialNoise_;
    mcl3d::IMU imu_;

    ros::Time mclPoseStamp_;

    double transformTolerance_;
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;
    bool broadcastTF_, useOdomTF_;

    std::string logFile_;
    bool writeLog_;

public:
    MCLNode(void):
        nh_("~"),
        mapFrame_("map"),
        odomFrame_("odom"),
        baseLinkFrame_("base_link"),
        laserFrame_("laser"),
        optPoseFrame_("opt_pose"),
        transformTolerance_(0.0),
        broadcastTF_(true),
        useOdomTF_(false),
        logFile_("/tmp/als_ros_3d_log.txt"),
        writeLog_(false)
    {
        // set subscribers
        std::string sensorPointsName = "/velodyne_points", imuName = "/imu/data", odomName = "/odom";
        bool useIMU = false, useOdom = false, useInitialPoseCB = true;
        nh_.param("sensor_points_name", sensorPointsName, sensorPointsName);
        nh_.param("imu_name", imuName, imuName);
        nh_.param("odom_name", odomName, odomName);
        nh_.param("use_imu", useIMU, useIMU);
        nh_.param("use_odom", useOdom, useOdom);
        nh_.param("use_initial_pose_cb", useInitialPoseCB, useInitialPoseCB);
        nh_.param("use_odom_tf", useOdomTF_, useOdomTF_);
        sensorPointsSub_ = nh_.subscribe(sensorPointsName, 1, &MCLNode::sensorPointsCB, this);
        if (useIMU)
            imuSub_ = nh_.subscribe(imuName, 1, &MCLNode::imuCB, this);
        if (useOdom || useOdomTF_)
            odomSub_ = nh_.subscribe(odomName, 1, &MCLNode::odomCB, this);
        if (useInitialPoseCB)
            initialPoseSub_ = nh_.subscribe("/initialpose", 1, &MCLNode::initialPoseCB, this);

        // set publishers
        std::string particlesName = "/particles", optParticlesName = "/optimized_particles";
        std::string poseName = "/mcl_pose", optPoseName = "/opt_pose";
        std::string alignedPointsOptName = "/aligned_points_opt";
        std::string mapPointsName = "/df_map_points";
        nh_.param("particles_name", particlesName, particlesName);
        nh_.param("opt_particles_name", optParticlesName, optParticlesName);
        nh_.param("pose_name", poseName, poseName);
        nh_.param("opt_pose_name", optPoseName, optPoseName);
        nh_.param("aligned_points_opt_name", alignedPointsOptName, alignedPointsOptName);
        nh_.param("map_points_name", mapPointsName, mapPointsName);
        particlesPub_ = nh_.advertise<geometry_msgs::PoseArray>(particlesName, 1);
        optParticlesPub_ = nh_.advertise<geometry_msgs::PoseArray>(optParticlesName, 1);
        posePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(poseName, 1);
        optPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(optPoseName, 1);
        alignedPointsOptPub_ = nh_.advertise<sensor_msgs::PointCloud>(alignedPointsOptName, 1);
        mapPointsPub_ = nh_.advertise<sensor_msgs::PointCloud>(mapPointsName, 1, this);

        // read tf frame names
        nh_.param("map_frame", mapFrame_, mapFrame_);
        nh_.param("odom_frame", odomFrame_, odomFrame_);
        nh_.param("base_link_frame", baseLinkFrame_, baseLinkFrame_);
        nh_.param("laser_frame", laserFrame_, laserFrame_);
        nh_.param("opt_pose_frame", optPoseFrame_, optPoseFrame_);

        // set localization mode
        int localizationMode = 0, measurementModelType = 3;
        nh_.param("localization_mode", localizationMode, localizationMode);
        nh_.param("measurement_model_type", measurementModelType, measurementModelType);
        mcl_.setLocalizationMode(localizationMode);
        mcl_.setMeasurementModelType(measurementModelType);

        // set number of particles
        int particleNum = 500;
        nh_.param("particle_num", particleNum, particleNum);
        mcl_.setParticleNum(particleNum);

        // set number of points used for likelihood calculation
        int sensorPointsNum = 200;
        nh_.param("setSensorPointsNum", sensorPointsNum, sensorPointsNum);
        mcl_.setSensorPointsNum(sensorPointsNum);

        // set voxel leaf size
        double voxelLeafSize = 1.0;
        nh_.param("voxel_leaf_size", voxelLeafSize, voxelLeafSize);
        mcl_.setVoxelLeafSize(voxelLeafSize);

        // set pose related parameters
        std::vector<double> initialPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> initialNoise = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
        std::vector<double> baseLink2Laser = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        nh_.param("initial_pose", initialPose, initialPose);
        nh_.param("initial_noise", initialNoise, initialNoise);
        nh_.param("base_link_2_laser", baseLink2Laser, baseLink2Laser);
        initialPose[3] *= M_PI / 180.0;
        initialPose[4] *= M_PI / 180.0;
        initialPose[5] *= M_PI / 180.0;
        baseLink2Laser[3] *= M_PI / 180.0;
        baseLink2Laser[4] *= M_PI / 180.0;
        baseLink2Laser[5] *= M_PI / 180.0;
        initialPose_.setPose(initialPose[0], initialPose[1], initialPose[2],
            initialPose[3], initialPose[4], initialPose[5]);
        initialNoise_.setPose(initialNoise[0], initialNoise[1], initialNoise[2],
            initialNoise[3], initialNoise[4], initialNoise[5]);
        mcl3d::Pose baseLink2Laser_(baseLink2Laser[0], baseLink2Laser[1], baseLink2Laser[2],
            baseLink2Laser[3], baseLink2Laser[4], baseLink2Laser[5]);
        mcl_.setInitialPose(initialPose_);
        mcl_.setBaseLink2Laser(baseLink2Laser_);
        mcl_.initializeParticles(initialPose_, initialNoise_);

        // set IMU parameters
        if (useIMU) {
            imu_.init();
            double imuSampleFreq = 100.0, AHRSFilterKp = 0.0, AHRSFilterKi = 0.0;
            nh_.param("imu_sample_freq", imuSampleFreq, imuSampleFreq);
            nh_.param("ahrs_filter_kp", AHRSFilterKp, AHRSFilterKp);
            nh_.param("ahrs_filter_ki", AHRSFilterKi, AHRSFilterKi);
            imu_.setSampleFreq(imuSampleFreq);
            imu_.setAHRSFilterGains(AHRSFilterKp, AHRSFilterKi);
        }

        // set optimization parameters

        // set measurement model parameters
        double zHit = 0.9, zRand = 0.05, zMax = 0.05;
        double varHit = 0.01, unknownLambda = 0.001, rangeReso = 0.1, rangeMax = 120.0;
        nh_.param("z_hit", zHit, zHit);
        nh_.param("z_rand", zRand, zRand);
        nh_.param("z_max", zMax, zMax);
        nh_.param("var_hit", varHit, varHit);
        nh_.param("unknown_lambda", unknownLambda, unknownLambda);
        nh_.param("range_reso", rangeReso, rangeReso);
        nh_.param("range_max", rangeMax, rangeMax);
        mcl_.setMeasurementModelParameters(zHit, zRand, zMax, varHit, unknownLambda, rangeReso, rangeMax);

        std::vector<double> odomNoise = {1.0, 0.1, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 1.0, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 1.0, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 1.0, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 1.0, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 0.1, 1.0};
        nh_.param("odom_noise", odomNoise, odomNoise);
        mcl_.setOdomNoise(odomNoise);

        bool useLinearInterpolation = true;
        nh_.param("use_linear_interpolation", useLinearInterpolation, useLinearInterpolation);
        mcl_.setUseLinearInterpolation(useLinearInterpolation);

        double randomParticleRate = 0.1;
        double resampleThreshold = 0.5;
        std::vector<double> resampleNoise = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
        nh_.param("random_particle_rate", randomParticleRate, randomParticleRate);
        nh_.param("resample_threshold", resampleThreshold, resampleThreshold);
        nh_.param("resample_noise", resampleNoise, resampleNoise);
        mcl3d::Pose resampleNoise_(resampleNoise[0], resampleNoise[1], resampleNoise[2],
            resampleNoise[3], resampleNoise[4], resampleNoise[5]);
        mcl_.setRandomParticleRate(randomParticleRate);
        mcl_.setResampleThreshold(resampleThreshold);
        mcl_.setResampleNoise(resampleNoise_);

        int optMaxIterNum = 30;
        double optMaxError = 1.0, convergenceThreshold = 0.02;
        nh_.param("opt_max_iter_num", optMaxIterNum, optMaxIterNum);
        nh_.param("opt_max_error", optMaxError, optMaxError);
        nh_.param("convergence_threshold", convergenceThreshold, convergenceThreshold);
        mcl_.setOptMaxIterNum(optMaxIterNum);
        mcl_.setOptMaxError(optMaxError);
        mcl_.setConvergenceThreshold(convergenceThreshold);

        // set fusion parameters
        int optParticleNum = 100;
        double optPoseCovCoef = 1.0, gmmPosVar = 0.3, gmmAngVar = 0.1;
        nh_.param("optimized_particle_num", optParticleNum, optParticleNum);
        nh_.param("optimized_pose_cov_coef", optPoseCovCoef, optPoseCovCoef);
        nh_.param("gmm_postion_var", gmmPosVar, gmmPosVar);
        nh_.param("gmm_angle_var", gmmAngVar, gmmAngVar);
        mcl_.setOptParticlsNum(optParticleNum);
        mcl_.setOptPoseCovCoef(optPoseCovCoef);
        mcl_.setGMMPosVar(gmmPosVar);
        mcl_.setGMMAngVar(gmmAngVar);

        // read tf parameters
        nh_.param("transform_tolerance", transformTolerance_, transformTolerance_);
        nh_.param("broadcast_tf", broadcastTF_, broadcastTF_);
        nh_.param("use_odom_tf", useOdomTF_, useOdomTF_);

        // initialization for MCL
        if (!mcl_.checkParameters()) {
            ROS_ERROR("Incorrect parameters are given for MCL.");
            exit(1);
        }

        // load map or set map points subscriber
        std::string mapYamlFile = "/home/akai/Dropbox/git/git_ros_ws/src/als_ros_3d/data/dist_map_mcl_3dl.yaml";
        nh_.param("map_yaml_file", mapYamlFile, mapYamlFile);
        ROS_INFO("The given map yaml file is %s. Start map loading.", mapYamlFile.c_str());
        if (!mcl_.loadDistanceMap(mapYamlFile)) {
            ROS_ERROR("Cannot read map yaml file -> %s", mapYamlFile.c_str());
            exit(1);
        }

        std::vector<mcl3d::Point> mapPoints = mcl_.getMapPoints();
        sensor_msgs::PointCloud mapPointsMsg;
        mapPointsMsg.header.frame_id = mapFrame_;
        mapPointsMsg.points.resize((int)mapPoints.size());
        for (int i = 0; i < (int)mapPoints.size(); ++i) {
            geometry_msgs::Point32 p;
            p.x = mapPoints[i].getX();
            p.y = mapPoints[i].getY();
            p.z = mapPoints[i].getZ();
            mapPointsMsg.points[i] = p;
        }
        mapPointsMsg.header.stamp = ros::Time::now();
        mapPointsPub_.publish(mapPointsMsg);

        nh_.param("log_file", logFile_, logFile_);
        nh_.param("write_log", writeLog_, writeLog_);

        ROS_INFO("Initialization for MCL has been done.");
    }

    // main localization process callback
    void sensorPointsCB(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        static double totalTime = 0.0;
        static int count = 0;

        mclPoseStamp_ = msg->header.stamp;
        pcl::PointCloud<pcl::PointXYZ> sensorPointsTmp;
        pcl::fromROSMsg(*msg, sensorPointsTmp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints(new pcl::PointCloud<pcl::PointXYZ>);
        *sensorPoints = sensorPointsTmp;

        mcl_.updatePoses();
        std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        mcl_.calculateLikelihoodsByMeasurementModel(sensorPoints);
        mcl_.optimizeMeasurementModel(sensorPoints);
        mcl_.resampleParticles1();
        mcl_.resampleParticles2();
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
        totalTime += time;
        count++;
        printf("time = %lf, average = %lf\n", time, totalTime / (double)count);
        mcl_.printMCLResult();
        publishROSMessages();
        broadcastTF();
        writeLog();
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr &msg) {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (norm > 0.99) {
            tf::Quaternion q(qx, qy, qz, qw);
            double roll, pitch, yaw;
            tf::Matrix3x3 mat(q);
            mat.getRPY(roll, pitch, yaw);
            imu_.setRPY(roll, pitch, yaw);
        } else {
            imu_.setAcceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu_.setAngularVelocities(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            imu_.updateOrientation();
            // imu_.printIMUResult();
        }
        mcl_.setIMU(imu_);
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mcl3d::Pose odomPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z, roll, pitch, yaw);
        mcl3d::Pose odomVel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
            msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        mcl_.setOdomPose(odomPose);
        mcl_.setOdomVelocities(odomVel);
    }

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mcl3d::Pose initialPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z, roll, pitch, yaw);
        mcl_.setInitialPose(initialPose);
        mcl_.initializeParticles(initialPose, initialNoise_);

/*
        if (estimateReliability_)
            resetReliabilities();
 */
    }

    void publishROSMessages(void) {
        // particles
        std::vector<mcl3d::Particle> particles = mcl_.getParticles();
        geometry_msgs::PoseArray particlesPoses;
        particlesPoses.header.frame_id = mapFrame_;
        particlesPoses.header.stamp = mclPoseStamp_;
        particlesPoses.poses.resize((int)particles.size());
        for (int i = 0; i < (int)particles.size(); ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = particles[i].getX();
            pose.position.y = particles[i].getY();
            pose.position.z = particles[i].getZ();
            tf::Quaternion q = tf::createQuaternionFromRPY(particles[i].getRoll(), particles[i].getPitch(), particles[i].getYaw());
            quaternionTFToMsg(q, pose.orientation);
            particlesPoses.poses[i] = pose;
        }
        particlesPub_.publish(particlesPoses);

        // optimized particles
        std::vector<mcl3d::Particle> optParticles = mcl_.getOptParticles();
        geometry_msgs::PoseArray optParticlesPoses;
        optParticlesPoses.header.frame_id = mapFrame_;
        optParticlesPoses.header.stamp = mclPoseStamp_;
        optParticlesPoses.poses.resize((int)optParticles.size());
        for (int i = 0; i < (int)optParticles.size(); ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = optParticles[i].getX();
            pose.position.y = optParticles[i].getY();
            pose.position.z = optParticles[i].getZ();
            tf::Quaternion q = tf::createQuaternionFromRPY(optParticles[i].getRoll(), optParticles[i].getPitch(), optParticles[i].getYaw());
            quaternionTFToMsg(q, pose.orientation);
            optParticlesPoses.poses[i] = pose;
        }
        optParticlesPub_.publish(optParticlesPoses);

        // mcl and optimized poses
        geometry_msgs::PoseWithCovarianceStamped mclPoseMsg, optPoseMsg;
        mcl3d::Pose mclPose = mcl_.getMCLPose();
        mcl3d::Pose optPose = mcl_.getOptPose();

        mclPoseMsg.header.frame_id = mapFrame_;
        mclPoseMsg.header.stamp = mclPoseStamp_;
        mclPoseMsg.pose.pose.position.x = mclPose.getX();
        mclPoseMsg.pose.pose.position.y = mclPose.getY();
        mclPoseMsg.pose.pose.position.z = mclPose.getZ();
        tf::Quaternion mclQuat = tf::createQuaternionFromRPY(mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw());
        quaternionTFToMsg(mclQuat, mclPoseMsg.pose.pose.orientation);

        optPoseMsg.header.frame_id = mapFrame_;
        optPoseMsg.header.stamp = mclPoseStamp_;
        optPoseMsg.pose.pose.position.x = optPose.getX();
        optPoseMsg.pose.pose.position.y = optPose.getY();
        optPoseMsg.pose.pose.position.z = optPose.getZ();
        tf::Quaternion optQuat = tf::createQuaternionFromRPY(optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
        quaternionTFToMsg(optQuat, optPoseMsg.pose.pose.orientation);

        std::vector<std::vector<double>> poseCov = mcl_.getPoseCovariance();
        std::vector<std::vector<double>> optPoseCov = mcl_.getOptPoseCovariance();
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                mclPoseMsg.pose.covariance[j *6 + i] = poseCov[i][j];
                optPoseMsg.pose.covariance[j *6 + i] = optPoseCov[i][j];
            }
        }
        posePub_.publish(mclPoseMsg);
        optPosePub_.publish(optPoseMsg);

        // aligned points
        pcl::PointCloud<pcl::PointXYZ> alignedPointsOpt = mcl_.getAlignedPointsOpt();
        sensor_msgs::PointCloud alignedPointsOptMsg;
        alignedPointsOptMsg.points.resize((int)alignedPointsOpt.points.size());
        for (int i = 0; i < (int)alignedPointsOpt.points.size(); ++i) {
            geometry_msgs::Point32 p;
            p.x = alignedPointsOpt.points[i].x;
            p.y = alignedPointsOpt.points[i].y;
            p.z = alignedPointsOpt.points[i].z;
            alignedPointsOptMsg.points[i] = p;
        }
        alignedPointsOptMsg.header.frame_id = laserFrame_;
        alignedPointsOptMsg.header.stamp = mclPoseStamp_;
        alignedPointsOptPub_.publish(alignedPointsOptMsg);
    }

    void broadcastTF(void) {
        if (!broadcastTF_)
            return;

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        geometry_msgs::Pose poseOnMap;
        poseOnMap.position.x = mclPose.getX();
        poseOnMap.position.y = mclPose.getY();
        poseOnMap.position.z = mclPose.getZ();
        tf::Quaternion mclQuat = tf::createQuaternionFromRPY(mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw());
        quaternionTFToMsg(mclQuat, poseOnMap.orientation);
        tf2::Transform map2baseTrans;
        tf2::convert(poseOnMap, map2baseTrans);

        if (useOdomTF_) {
            // make TF tree as map -> odom -> base_link -> laser
            mcl3d::Pose odomPose = mcl_.getOdomPose();
            geometry_msgs::Pose poseOnOdom;
            poseOnOdom.position.x = odomPose.getX();
            poseOnOdom.position.y = odomPose.getY();
            poseOnOdom.position.z = odomPose.getZ();
            tf::Quaternion odomQuat = tf::createQuaternionFromRPY(odomPose.getRoll(), odomPose.getPitch(), odomPose.getYaw());
            quaternionTFToMsg(odomQuat, poseOnOdom.orientation);
            tf2::Transform odom2baseTrans;
            tf2::convert(poseOnOdom, odom2baseTrans);

            tf2::Transform map2odomTrans = map2baseTrans * odom2baseTrans.inverse();
            ros::Time transformExpiration = (mclPoseStamp_ + ros::Duration(transformTolerance_));
            geometry_msgs::TransformStamped map2odomStampedTrans;
            map2odomStampedTrans.header.stamp = transformExpiration;
            map2odomStampedTrans.header.frame_id = mapFrame_;
            map2odomStampedTrans.child_frame_id = odomFrame_;
            tf2::convert(map2odomTrans, map2odomStampedTrans.transform);
            tfBroadcaster_.sendTransform(map2odomStampedTrans);
        } else {
            // make TF tree as map -> base_link -> laser
            geometry_msgs::TransformStamped map2baseStampedTrans;
            map2baseStampedTrans.header.stamp = mclPoseStamp_;
            map2baseStampedTrans.header.frame_id = mapFrame_;
            map2baseStampedTrans.child_frame_id = baseLinkFrame_;
            tf2::convert(map2baseTrans, map2baseStampedTrans.transform);
            tfBroadcaster_.sendTransform(map2baseStampedTrans);
        }

        // optimized pose
        mcl3d::Pose optPose = mcl_.getOptPose();
        geometry_msgs::Pose optPoseOnMap;
        optPoseOnMap.position.x = optPose.getX();
        optPoseOnMap.position.y = optPose.getY();
        optPoseOnMap.position.z = optPose.getZ();
        tf::Quaternion optQuat = tf::createQuaternionFromRPY(optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
        quaternionTFToMsg(optQuat, optPoseOnMap.orientation);
        tf2::Transform map2optPoseTrans;
        tf2::convert(optPoseOnMap, map2optPoseTrans);
        geometry_msgs::TransformStamped map2optPoseStampedTrans;
        map2optPoseStampedTrans.header.stamp = mclPoseStamp_;
        map2optPoseStampedTrans.header.frame_id = mapFrame_;
        map2optPoseStampedTrans.child_frame_id = optPoseFrame_;
        tf2::convert(map2optPoseTrans, map2optPoseStampedTrans.transform);
        tfBroadcaster_.sendTransform(map2optPoseStampedTrans);
    }

    void writeLog(void) {
        if (!writeLog_)
            return;

        static FILE *fp;
        if (fp == NULL) {
            fp = fopen(logFile_.c_str(), "w");
            if (fp == NULL) {
                fprintf(stderr, "Cannot open %s\n", logFile_.c_str());
                return;
            }
        }

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        mcl3d::Pose optPose = mcl_.getOptPose();
        fprintf(fp, "%lf "
            "%lf %lf %lf %lf %lf %lf "
            "%lf %lf %lf %lf %lf %lf\n",
            mclPoseStamp_.toSec(),
            mclPose.getX(), mclPose.getY(), mclPose.getZ(), mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw(),
            optPose.getX(), optPose.getY(), optPose.getZ(), optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
    }

    void spin(void) {
        ros::Rate loopRate(20.0);
        while (ros::ok()) {
            ros::spinOnce();
            loopRate.sleep();
        }
    }
}; // class MCLNode

int main(int argc, char **argv) {
    ros::init(argc, argv, "als_ros_3d");
    MCLNode node;
    node.spin();
    return 0;
}
