#include <opencv2/imgproc/types_c.h>
#include "ViewController.hpp"

ViewController* ViewController::instance = nullptr;
ASensorEventQueue* ViewController::accelerometerEventQueue = nullptr;
ASensorEventQueue* ViewController::gyroscopeEventQueue = nullptr;

ViewController::ViewController() {
    LOGI("ViewController Constructor");
    this->instance = this;
}

ViewController::~ViewController() {
    LOGI("ViewController Destructor");
}

void ViewController::viewDidLoad() {
    isCapturing = false;

    if (!feature_tracker)
        feature_tracker = new FeatureTracker();

    // Give projection variance
    vins.setIMUModel();

    /**************************************** Initialize all the threads ****************************************/
    mainLoop = std::thread(&ViewController::run, this);

    saveData = std::thread(&ViewController::saveDataLoop, this);

    // TODO: Fix the loop closure functionality.
    if (DETECT_LOOP_CLOSURES)
    {
        // Loop closure thread
        loop_thread = std::thread(&ViewController::loopDetectionLoop, this);

        globalLoopThread = std::thread(&ViewController::globalPoseGraphLoop, this);
    }

    /************************************Device and iOS version check************************************/
    bool deviceCheck = setGlobalParam(deviceName());
    if(!deviceCheck)
    {
        LOGE("Device not supported");
    }

    vins.setExtrinsic();
    vins.setIMUModel();
    featuretracker.vins_pnp.setExtrinsic();
    vinsPnP::setIMUModel();

    /*********************************************Start VINS*******************************************/
    imuStartUpdate(); // [self imuStartUpdate];
    isCapturing = true;
    // TODO: Move mainLoopInit here

    frameSize = cv::Size(videoWidth, videoHeight);
    LOGI("Init successful");
}

void ViewController::processImage(cv::Mat &image, double timestamp, bool isScreenRotated) {
    TS(ViewController_processImage);

    if (isCapturing) {
        assert(image.size() == frameSize);

        if (latest_imu_time <= 0) {
            LOGI("IMU Timestamp negative: %lf, abort processImage()", latest_imu_time);
            if(isScreenRotated)
                cv::rotate(image, image, cv::ROTATE_180);
            return;
        }

        if (!imgDataBuf.empty()) {
            LOGI("!imgDataBuf.empty(), abort processImage()");
            return;
        }

        shared_ptr<ImageMeta> img_metadata(new ImageMeta());
        img_metadata->timestamp = timestamp;

        cv::Mat grayscale_image;
        cv::cvtColor(image, grayscale_image, CV_RGBA2GRAY);
        cv::Mat image_with_features;
        cv::Mat equalized_image;

        // Contrast Limited Adaptive Histogram Equalization
        cv::Ptr<cv::CLAHE> contrast_equalizer = cv::createCLAHE();

        // Set clip limit to limit noise in regions with similar contrast.
        // (https://en.wikipedia.org/wiki/Adaptive_histogram_equalization#Contrast_Limited_AHE)
        contrast_equalizer->setClipLimit(3);
        contrast_equalizer->apply(grayscale_image, equalized_image);

        // solved_features and solved_vins are modified by another thread
        // (process), mutexes prevents double reads/writes.
        m_depth_feedback_mutex.lock();
        featuretracker.solved_features = solved_features;
        featuretracker.solved_vins = solved_vins;
        m_depth_feedback_mutex.unlock();

        m_imu_feedback_mutex.lock();
        // Get all IMU measurements between last frame and this frame.
        featuretracker.imu_msgs = getImuMeasurements(img_metadata->timestamp);
        m_imu_feedback_mutex.unlock();

        if (this->saveDir.length() > 0) {
            auto filename = this->saveDir + to_string(timestamp);
            FILE *file = fopen(filename.c_str(), "w+");

            if (file != NULL) {
//                fputs("timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ\n", file);
                int size = featuretracker.imu_msgs.size();
                fwrite( &size, 1, sizeof(size), file) ;
                for (auto el : featuretracker.imu_msgs) {
                    double xAcc = el.acc.x(), yAcc = el.acc.y(), zAcc = el.acc.z();
                    double xGyr = el.gyr.x(), yGyr = el.gyr.y(), zGyr = el.gyr.z();

                    fwrite( &xAcc, 1, sizeof(xAcc), file);
                    fwrite( &yAcc, 1, sizeof(yAcc), file);
                    fwrite( &zAcc, 1, sizeof(zAcc), file);

                    fwrite( &xGyr, 1, sizeof(xGyr), file);
                    fwrite( &yGyr, 1, sizeof(yGyr), file);
                    fwrite( &zGyr, 1, sizeof(zGyr), file);

//                    fprintf(file, "%f,%f,%f,%f,%f,%f,%f", el.timestamp, el.acc.x(), el.acc.y(), el.acc.z(), el.gyr.x(), el.gyr.y(), el.gyr.z());
                }

                size_t sizeInBytes = image.total() * image.elemSize();
                fwrite(image.data, 1, sizeInBytes, file);
                fflush(file);
                fclose(file);
            }
        }

        vector<Point2f> good_pts;
        vector<double> track_len;

        // Perspective n point
        featuretracker.use_pnp = USE_PNP;

        // This performs the actual feature tracking. Using the last available frame,
        // it will try to match up any movement between that frame and this one.
        featuretracker.readImage(
            equalized_image,
            image_with_features,
            frame_cnt,
            good_pts,
            track_len,
            img_metadata->timestamp,
            pnp_P,
            pnp_R,
            vins.solver_flag == VINS::NON_LINEAR
        );


        // This is the 10Hz Corner Detection (Feature Detection), not
        // the 30Hz Feature Detection(Feature Tracking) done earlier.
        // -> it is only providing the info and notifying the main thread
        if (featuretracker.img_cnt == 0) { // True every FREQ (3) frames
            //img_metadata callback
            img_metadata->point_clouds = featuretracker.image_msg;

            m_feature_and_IMU_buffer_mutex.lock();
            img_msg_buf.push(img_metadata);
            m_feature_and_IMU_buffer_mutex.unlock();
            con.notify_one();

            if (image_cache_enabled) {
                image_data_cache.header = img_metadata->timestamp;
                image_data_cache.image = image.clone();
                image_pool.push(image_data_cache);
            }

            if (DETECT_LOOP_CLOSURES) {
                m_image_buf_loop.lock();
                cv::Mat loop_image = grayscale_image.clone();
                image_buf_loop.push(make_pair(loop_image, img_metadata->timestamp));
                if(image_buf_loop.size() > WINDOW_SIZE)
                    image_buf_loop.pop();
                m_image_buf_loop.unlock();
            }
        }

        featuretracker.img_cnt = (featuretracker.img_cnt + 1) % FREQ;

        for (int i = 0; i < good_pts.size(); i++) {
            cv::circle(
                image,
                good_pts[i],
                0,
                cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]),
                7
            ); //BGR
        }

        if (image_cache_enabled) {
            //use aligned VINS and image
            if (!vins_pool.empty() && !image_pool.empty()) {
                while(vins_pool.size() > 1) {
                    vins_pool.pop();
                }

                while (!image_pool.empty() && image_pool.front().header < vins_pool.front().header) {
                    image_pool.pop();
                }

                if (!vins_pool.empty() && !image_pool.empty()) {
                    latest_image = image_pool.front().image;
                    latest_P = vins_pool.front().P;
                    latest_R = vins_pool.front().R;
                    image = latest_image.clone();
                }
            }
            else if (!image_pool.empty())
            {
                if(image_pool.size() > 10)
                    image_pool.pop();
            }
        }

        if (USE_PNP) {
            latest_P = pnp_P.cast<float>();
            latest_R = pnp_R.cast<float>();
        }

        // Controlled by toggle switch
        if (ui_main || !start_show || vins.solver_flag != VINS::NON_LINEAR) {
            // Show Image and AR

            if (vins.solver_flag == VINS::NON_LINEAR && start_show) {
                cv::Mat tmp;
                vins.drawresult.startInit = true;

                vins.drawresult.drawAR(
                    vins.imageAI,
                    vins.correct_point_cloud,
                    latest_P,
                    latest_R
                );

                cv::cvtColor(image, tmp, CV_RGBA2RGB);
                cv::Mat mask;

                cv::Mat imageAI = vins.imageAI;
                if (!imageAI.empty())
                    cv::cvtColor(imageAI, mask, CV_RGB2GRAY);

                imageAI.copyTo(tmp, mask);

                cv::cvtColor(tmp, image, CV_RGB2RGBA);
                if (isScreenRotated)
                    cv::rotate(image, image, cv::ROTATE_180);
            } else {
                if (isScreenRotated)
                    cv::rotate(image, image, cv::ROTATE_180);
            }
        } else {
            // Show the camera view with a 3rd person view

            vins.drawresult.pose.clear();
            vins.drawresult.pose = keyframe_database.refine_path;
            vins.drawresult.segment_indexs = keyframe_database.segment_indexs;
            // Render Camera following current Position
            vins.drawresult.origin_w[0] = (float) vins.correct_Ps[frame_cnt][0];
            vins.drawresult.origin_w[1] = (float) vins.correct_Ps[frame_cnt][1];

            vins.drawresult.radius = virtualCamDistance;
            // Rotation:
            vins.drawresult.theta = 75; // around horizontal - pitch
            vins.drawresult.phy = 89; // around vertical - yaw
            vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud,
                                         vins.correct_Rs, vins.correct_Ps, box_in_trajectory);

            cv::Mat tmp2 = vins.image_show;

            // Scale down the viewport to show it in the bottom left of the screen.
            cv::Mat down_origin_image;
            cv::resize(image, down_origin_image, cv::Size(150, 200));
            cv::rotate(down_origin_image, down_origin_image, cv::ROTATE_90_CLOCKWISE);
            cv::cvtColor(down_origin_image, down_origin_image, CV_RGBA2RGB);

            if (!isScreenRotated)
                cv::rotate(down_origin_image, down_origin_image, cv::ROTATE_180);

            cv::Mat imageROI;
            imageROI = tmp2(cv::Rect(10,COL - down_origin_image.rows- 10, down_origin_image.cols,down_origin_image.rows));
            down_origin_image.copyTo(imageROI);
            cv::rotate(tmp2, image, cv::ROTATE_90_CLOCKWISE);
            cv::cvtColor(image, image, CV_RGB2RGBA);
        }

        // prints information about how long the visualization took in ms
    } else {
        // Not capturing, means not started yet
        LOGI("not capturing");
        
        if(isScreenRotated)
            cv::rotate(image, image, cv::ROTATE_180);
    }
    TE(ViewController_processImage);
}

std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> ViewController::getMeasurements() {
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true) {
        if (imu_msg_buf.empty() || img_msg_buf.empty())
            return measurements;

        if (img_msg_buf.front()->timestamp >= imu_msg_buf.back()->header)
        {
            __android_log_print(ANDROID_LOG_INFO, "VINS", "wait for imu, only should happen at the beginning");
            return measurements;
        }

        if (imu_msg_buf.front()->header >= img_msg_buf.front()->timestamp)
        {
            __android_log_print(ANDROID_LOG_INFO, "VINS", "throw img, only should happen at the beginning");
            img_msg_buf.pop();
            continue;
        }

        ImgConstPtr img_msg = img_msg_buf.front(); // auto img_msg = img_msg_buf.front();

        img_msg_buf.pop();

        std::vector<ImuConstPtr> IMUs;
        while (imu_msg_buf.front()->header <= img_msg->timestamp)
        {
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }

        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

vector<IMUMsgLocal> ViewController::getImuMeasurements(double header) {
    vector<IMUMsgLocal> imu_measurements;
    static double last_header = -1; // STATIC only happens once
    if (last_header < 0 || local_imu_msg_buf.empty())
    {
        last_header = header;
        return imu_measurements;
    }

    while (!local_imu_msg_buf.empty() && local_imu_msg_buf.front().timestamp <= last_header)
        local_imu_msg_buf.pop();

    while (!local_imu_msg_buf.empty() && local_imu_msg_buf.front().timestamp <= header)
    {
        imu_measurements.emplace_back(local_imu_msg_buf.front());
        local_imu_msg_buf.pop();
    }

    last_header = header;
    return imu_measurements;
}

void ViewController::send_imu(const ImuConstPtr &imu_msg) {
    NSTimeInterval t = imu_msg->header;
    if (current_time < 0)
        current_time = t;
    double dt = (t - current_time);
    current_time = t;

    // should that be some bias we set?
    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    // TODO: ??? Some magic thing happening or are we ALWAYS subtracting 0? 
    double dx = imu_msg->acc.x() - ba[0];
    double dy = imu_msg->acc.y() - ba[1];
    double dz = imu_msg->acc.z() - ba[2];

    double rx = imu_msg->gyr.x() - bg[0];
    double ry = imu_msg->gyr.y() - bg[1];
    double rz = imu_msg->gyr.z() - bg[2];

    vins.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void ViewController::run() {
    _condition.lock();

    while (!mainLoop_isCancelled) {
        LOGI("THREAD: Main Thread(run): process()");
        process();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        LOGI("THREAD: Main Thread iteration done");
    }

    _condition.unlock(); //[_condition unlock];
}

void ViewController::process() {
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(m_feature_and_IMU_buffer_mutex);

    con.wait(lk, [&] {
        measurements = getMeasurements();
        return !measurements.empty();
    });

    lk.unlock();
    waiting_lists = measurements.size();
    for (auto &measurement : measurements)
    {
        TS(IMU_Data);
        for(auto &imu_msg : measurement.first)
        {
            send_imu(imu_msg);
        }
        TE(IMU_Data);

        auto img_msg = measurement.second;
        map<int, Vector3d> image = img_msg->point_clouds;
        //__android_log_print(ANDROID_LOG_INFO, APPNAME, "Image timestamp = %lf",img_msg->header);
        double header = img_msg->timestamp;
        TS(process_image);
        vins.processImage(image,header,waiting_lists);
        TE(process_image);
        // unable to get same timing system as the sensor timestamps: systemUptime(); // [[NSProcessInfo processInfo] systemUptime];
        double time_now = latest_imu_time;
        double time_vins = vins.Headers[WINDOW_SIZE];
//        __android_log_print(ANDROID_LOG_INFO, APPNAME, "vins delay %lf", time_now - time_vins);

        //update feature position for front-end
        if (vins.solver_flag == vins.NON_LINEAR)
        {
            m_depth_feedback_mutex.lock();
            solved_vins.header = vins.Headers[WINDOW_SIZE - 1];
            solved_vins.Ba = vins.Bas[WINDOW_SIZE - 1];
            solved_vins.Bg = vins.Bgs[WINDOW_SIZE - 1];
            solved_vins.P = vins.correct_Ps[WINDOW_SIZE-1].cast<double>();
            solved_vins.R = vins.correct_Rs[WINDOW_SIZE-1].cast<double>();
            solved_vins.V = vins.Vs[WINDOW_SIZE - 1];
            Vector3d R_ypr = Utility::R2ypr(solved_vins.R);
            solved_features.clear();
            for (auto &it_per_id : vins.f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                if (it_per_id.solve_flag != 1)
                    continue;
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;

                ImageFeature tmp_feature;
                tmp_feature.id = it_per_id.feature_id;
                tmp_feature.position = vins.r_drift * vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.r_drift * vins.Ps[imu_i] + vins.t_drift;
                tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
                solved_features.push_back(tmp_feature);
            }
            m_depth_feedback_mutex.unlock();
        }

        if (image_cache_enabled)
        {
            //add state into vins buff for alignwith image
            if (vins.solver_flag == VINS::NON_LINEAR && start_show)
            {
                VinsDataCache vins_data_cache;
                vins_data_cache.header = vins.Headers[WINDOW_SIZE-1];
                vins_data_cache.P = vins.correct_Ps[WINDOW_SIZE-1];
                vins_data_cache.R = vins.correct_Rs[WINDOW_SIZE-1];
                vins_pool.push(vins_data_cache);
            }
            else if (vins.failure_occur)
            {
                vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
                segmentation_index++;
                keyframe_database.max_seg_index++;
                keyframe_database.cur_seg_index = keyframe_database.max_seg_index;

                while(!vins_pool.empty())
                    vins_pool.pop();
            }
        }
        /**
         *** start build keyframe database for loop closure
         **/
        if (DETECT_LOOP_CLOSURES)
        {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR)
                first_frame = true;
            if (vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !image_buf_loop.empty())
            {
                first_frame = false;
                if(!first_frame && keyframe_freq % LOOP_FREQ == 0)
                {
                    keyframe_freq = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    m_image_buf_loop.lock();
                    while(!image_buf_loop.empty() && image_buf_loop.front().second < vins.Headers[WINDOW_SIZE - 2])
                    {
                        image_buf_loop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second);
                    if(vins.Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                    {
                        //TODO: parameterize this file path
                        const char *pattern_file = "/storage/emulated/0/Android/data/com.thkoeln.jmoeller.vins_mobile_androidport/files/brief_pattern.yml"; // [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];on
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
                        /*
                         ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                         ** and save the pointcloud to the keyframe for reprojection search correspondance
                         */
                        keyframe->buildKeyFrameFeatures(vins);
                        keyframe_database.add(keyframe);

                        global_frame_cnt++;
                    }
                    m_image_buf_loop.unlock();

                }
                else
                {
                    first_frame = false;
                }
                // update loop info
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    if(vins.Headers[i] == vins.front_pose.header)
                    {
                        KeyFrame* cur_kf = keyframe_database.getKeyframe(vins.front_pose.cur_index);
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0)
                        {
                            printf("Wrong loop\n");
                            cur_kf->removeLoop();
                            break;
                        }
                        cur_kf->updateLoopConnection(vins.front_pose.relative_t,
                                                     vins.front_pose.relative_q,
                                                     vins.front_pose.relative_yaw);
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for (int i = 0; i < keyframe_database.size(); i++)
                {
                    search_cnt++;
                    KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                    if (kf->header == vins.Headers[0])
                    {
                        kf->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        //update edge
                        // if loop happens in this frame, update pose graph;
                        if (kf->has_loop)
                        {
                            kf_global_index = kf->global_index;
                            start_global_optimization = true;
                        }
                        break;
                    }
                    else
                    {
                        if(search_cnt > 2 * WINDOW_SIZE)
                            break;
                    }
                }
                keyframe_freq++;
            }
        }
        waiting_lists--;

        //finish solve one frame
        showInputView();
        // [self performSelectorOnMainThread:@selector(showInputView) withObject:nil waitUntilDone:YES];
    }
}

void ViewController::loopDetectionLoop() {
    if (DETECT_LOOP_CLOSURES && loop_closure == nullptr) {
        __android_log_print(ANDROID_LOG_INFO, APPNAME, "loop start load voc");
        TS(load_voc);
        //TODO: parameterize this
        const char *voc_file = "/storage/emulated/0/Android/data/com.thkoeln.jmoeller.vins_mobile_androidport/files/brief_k10l6.bin"; // [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loop_closure = new LoopClosure(voc_file, COL, ROW);
        TE(load_voc);
        __android_log_print(ANDROID_LOG_INFO, APPNAME, "loop load voc finish");

        voc_init_ok = true;
    }
    while (!loop_thread_isCancelled) {
        if(!DETECT_LOOP_CLOSURES) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // [NSThread sleepForTimeInterval:0.5];
            continue;
        }

        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt) {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;

            cv::Mat current_image;
            current_image = cur_kf->image;

            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;

            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            
            TS(Feature_Retrieval);
            cur_kf->extractBrief(current_image);
            TE(Feature_Retrieval);
            
            printf("loop extract %lu feature\n", cur_kf->keypoints.size());
            TS(Loop_Detection);
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            TE(Loop_Detection);
            if(loop_succ) {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL) {
                    printf("NO such %dth frame in keyframe_database\n", old_index);
                    assert(false);
                }
                printf("loop succ with %drd image\n", old_index);
                assert(old_index!=-1);

                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;

                old_kf->getPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;

                if(measurements_old_norm.size()>MIN_LOOP_NUM) {
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    vins.retrive_pose_data = (retrive_data);

                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_old_index = old_index;
                }
            }
            cur_kf->image.release();
        }

        if(loop_succ)
            std::this_thread::sleep_for(std::chrono::seconds(2)); // [NSThread sleepForTimeInterval:2.0];
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

void ViewController::globalPoseGraphLoop() {
    while(!globalLoopThread_isCancelled) {
        if(start_global_optimization) {
            start_global_optimization = false;
            TS(globalPoseGraph);
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,
                                                        loop_correct_t,
                                                        loop_correct_r);
            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            TE(globalPoseGraph);
            std::this_thread::sleep_for(std::chrono::milliseconds(1170)); // [NSThread sleepForTimeInterval:1.17];
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // [NSThread sleepForTimeInterval:0.03];
    }
}

void ViewController::imuStopUpdate() {

    ASensorManager *sensorManager = ASensorManager_getInstance();
    assert(sensorManager != NULL);
    const ASensor *accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    const ASensor *gyroscope = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
    
    ASensorEventQueue_disableSensor(accelerometerEventQueue, accelerometer);
    ASensorEventQueue_disableSensor(gyroscopeEventQueue, gyroscope);
}

void ViewController::imuStartUpdate() const {

    ASensorManager *sensorManager = ASensorManager_getInstance();
    assert(sensorManager != NULL);

    ALooper* looper = ALooper_forThread();
    if(looper == NULL)
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    assert(looper != NULL);

    accelerometerEventQueue = ASensorManager_createEventQueue(sensorManager, looper,
                                                              LOOPER_ID_USER, process_imu_looper_events,
                                                              NULL);

    assert(accelerometerEventQueue != NULL);
    const ASensor *accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    assert(accelerometer != NULL);
    auto status = ASensorEventQueue_enableSensor(accelerometerEventQueue,
                                                 accelerometer);
    assert(status >= 0);
    status = ASensorEventQueue_setEventRate(accelerometerEventQueue,
                                            accelerometer,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    gyroscopeEventQueue = ASensorManager_createEventQueue(sensorManager, looper,
                                                          LOOPER_ID_USER, process_imu_sensor_events,
                                                          NULL);
    assert(gyroscopeEventQueue != NULL);
    const ASensor *gyroscope = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
    assert(gyroscope != NULL);
    status = ASensorEventQueue_enableSensor(gyroscopeEventQueue,
                                            gyroscope);
    assert(status >= 0);
    status = ASensorEventQueue_setEventRate(gyroscopeEventQueue,
                                            gyroscope,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    LOGI("IMU EventQueues initialized and started");
}

/*
 * Essentially, a null callback provided to the accelerometerEventQueue
 * for the looper. Wasn't taking an actual null callback due to a number
 * of factors. May need to find another solution.
 */
int ViewController::process_imu_looper_events(int fd, int events, void *data) {
    return 1;
}

int ViewController::process_imu_sensor_events(int fd, int events, void *data) {
    static ASensorEvent accelEvent;
    static double accelEventTimestamp = -1.0;
    ASensorEvent gyroEvent;
    
    while (ASensorEventQueue_getEvents(gyroscopeEventQueue, &gyroEvent, 1) > 0) {
        assert(gyroEvent.type == ASENSOR_TYPE_GYROSCOPE);


        double timeStampGyro = timeStampToSec(gyroEvent.timestamp);
        // The timestamp is the amount of time in seconds since the device booted.
        assert(timeStampGyro > 0);

        IMUMessage gyro_msg;
        gyro_msg.header = timeStampGyro;
        // in iOS and in Android the unit is rad/s
        gyro_msg.gyr << gyroEvent.uncalibrated_gyro.x_uncalib, //latestGyro.rotationRate.x,
                        gyroEvent.uncalibrated_gyro.y_uncalib, //latestGyro.rotationRate.y,
                        gyroEvent.uncalibrated_gyro.z_uncalib; //latestGyro.rotationRate.z;


                
        if (instance->gyro_buf.empty()) {
            LOGI("gyro interpolation buffer empty. should only happen once.");
            instance->gyro_buf.push_back(gyro_msg);
            instance->gyro_buf.push_back(gyro_msg);
            continue;
        }
        else if (gyro_msg.header <= instance->gyro_buf[1].header) {
            // Apparently events can be fired twice
            // Drop this event as it isn't more recent than the last one
            continue;
        }
        else {
            instance->gyro_buf[0] = instance->gyro_buf[1];
            instance->gyro_buf[1] = gyro_msg;
        }
        
        if(instance->imu_prepare < 10) {
            instance->imu_prepare++;
            continue;
        }
        
        while (accelEventTimestamp < instance->gyro_buf[0].header) {
//            LOGI("accelEventTimestamp < gyroEvent.timestamp: %lf < %lf", accelEventTimestamp , instance->gyro_buf[0].header);
            ssize_t numEvents;
            while((numEvents = ASensorEventQueue_getEvents(accelerometerEventQueue, &accelEvent, 1)) == 0) {
//                LOGI("having to wait for accl event");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            assert(numEvents == 1);
            assert(accelEvent.type == ASENSOR_TYPE_ACCELEROMETER);

            accelEventTimestamp = timeStampToSec(accelEvent.timestamp);
            
//            LOGI("IMU accl event timeStamp: %lf", timeStampAccl);
            shared_ptr<IMUMessage> acc_msg(new IMUMessage());
            acc_msg->header = accelEventTimestamp;
            // TODO: Apply a matrix multiplication to enable use of different coordinate systems
            // in Android the unit is m/s^2 in iOS it is g (9.8m/s^2)
            acc_msg->acc << accelEvent.acceleration.x,
                            accelEvent.acceleration.y,
                            accelEvent.acceleration.z;
            instance->cur_acc = acc_msg;
        }
//        LOGI("waited for accl event: %lf >= %lf", accelEventTimestamp, instance->gyro_buf[0].header);
        if(instance->gyro_buf[1].header < accelEventTimestamp){
            LOGE("having to wait for fitting gyro event"); // This should not happen if the frequency is the same
            continue;
        }


        //interpolation
        shared_ptr<IMUMessage> imu_msg(new IMUMessage());
//        LOGI("instance->cur_acc->header: \t\t%lf \ninstance->gyro_buf[0].header: \t%lf \ninstance->gyro_buf[1].header: \t%lf", instance->cur_acc->header, instance->gyro_buf[0].header, instance->gyro_buf[1].header);
        // TODO: Revert back to < not <=
        // This may be fixed when using the correct parameters for the phone
        if(instance->cur_acc->header >= instance->gyro_buf[0].header &&
        instance->cur_acc->header <= instance->gyro_buf[1].header) {
            imu_msg->header = instance->cur_acc->header;
//            imu_msg->header = (double)cv::getTickCount() / cv::getTickFrequency();
            imu_msg->acc = instance->cur_acc->acc;
            imu_msg->gyr = instance->gyro_buf[0].gyr +
                           (instance->gyro_buf[1].gyr - instance->gyro_buf[0].gyr) *
                           (instance->cur_acc->header - instance->gyro_buf[0].header) /
                           (instance->gyro_buf[1].header - instance->gyro_buf[0].header);
            //printf("imu gyro update %lf %lf %lf\n", instance.gyro_buf[0].header, imu_msg->header, instance.gyro_buf[1].header);
            //printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, instance.gyro_buf[0].gyr.x(), imu_msg->gyr.x(), instance.gyro_buf[1].gyr.x());
        }
        else {
            LOGE("imu error %lf %lf %lf\n", instance->gyro_buf[0].header, instance->cur_acc->header, instance->gyro_buf[1].header);
            continue;
        }
        
        
        //TODO: add playback and recording back in
        
//        LOGI("pushing new imu_msg & setting latest_imu_time: %lf \nAcc: %lf %lf %lf \nGyro: %lf %lf %lf", imu_msg->header,
//             imu_msg->acc[0], imu_msg->acc[1], imu_msg->acc[2],
//             imu_msg->gyr[0], imu_msg->gyr[1], imu_msg->gyr[2]);
        instance->latest_imu_time = imu_msg->header;

        //img_msg callback
        {
            IMUMsgLocal imu_msg_local;
            imu_msg_local.timestamp = imu_msg->header;
            imu_msg_local.acc = imu_msg->acc;
            imu_msg_local.gyr = imu_msg->gyr;

            instance->m_imu_feedback_mutex.lock();
            instance->local_imu_msg_buf.push(imu_msg_local);
            instance->m_imu_feedback_mutex.unlock();
        }
        instance->m_feature_and_IMU_buffer_mutex.lock();
        instance->imu_msg_buf.push(imu_msg);
        //__android_log_print(ANDROID_LOG_INFO, APPNAME, "IMU_buf timestamp %lf, acc_x = %lf",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
        instance->m_feature_and_IMU_buffer_mutex.unlock();
        instance->con.notify_one();
    }

    //should return 1 to continue receiving callbacks, or 0 to unregister                                                                                                                           
    return 1;
}

void ViewController::saveDataLoop() {
    while(!saveData_isCancelled)
    {
        if(!imgDataBuf.empty())
        {
            LOGI("saveDataLoop: Trying to save something");
            ImageData tmp_data;
            tmp_data = imgDataBuf.front();
            imgDataBuf.pop();
            recordImageTime(tmp_data); // [self recordImageTime:tmp_data];
            recordImage(tmp_data); // [self recordImage:tmp_data];
            imageDataIndex++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40)); // [NSThread sleepForTimeInterval:0.04];
    }
}

DeviceType ViewController::deviceName() {
//    return DeviceType::GalaxyS7;
    return DeviceType::Pixel6Pro;
}

/**
 * Sets info-texts that will later be displayed on screen
 * Reason:
 * The android system is extremely hard to convince to change ui elements from anywhere 
 * else than a specific jni call from the ui thread itself
 */
void ViewController::showInputView() {
    std::ostringstream oss;
    
    static bool finish_init = false;
    
    viewUpdateMutex.lock();
    if (vins.solver_flag != vins.NON_LINEAR) {
        finish_init = false;
        switch (vins.init_status) {
            case VINS::InitStatus::FAIL_IMU: // vins.FAIL_IMU:
                oss << "STA: FAIL_IMU"; // stringView = [NSString stringWithFormat:@"STA: FAIL_IMU"];
                break;
            case VINS::InitStatus::FAIL_PARALLAX:
                oss << "STA: FAIL_PARA"; // stringView =[NSString stringWithFormat:@"STA: FAIL_PARA"];
                break;
            case VINS::InitStatus::FAIL_RELATIVE:
                oss << "STA: FAIL_RELA"; // stringView =[NSString stringWithFormat:@"STA: FAIL_RELA"];
                break;
            case VINS::InitStatus::FAIL_SFM:
                oss << "STA: FAIL_SFM"; // stringView =[NSString stringWithFormat:@"STA: FAIL_SFM"];
                break;
            case VINS::InitStatus::FAIL_PNP:
                oss << "STA: FAIL_PNP"; // stringView =[NSString stringWithFormat:@"STA: FAIL_PNP"];
                break;
            case VINS::InitStatus::FAIL_ALIGN:
                oss << "STA: FAIL_ALIGN"; // stringView =[NSString stringWithFormat:@"STA: FAIL_ALIGN"];
                break;
            case VINS::InitStatus::FAIL_CHECK:
                oss << "STA: FAIL_COST"; // stringView =[NSString stringWithFormat:@"STA: FAIL_COST"];
                break;
            case VINS::InitStatus::SUCC:
                oss << "STA: SUCC!"; // stringView =[NSString stringWithFormat:@"STA: SUCC!"];
                break;
            default:
                oss << "STA: ";
                break;
        }

        tvXText = oss.str();
        oss.str(""); oss.clear();
        
        oss << "FAIL: " << vins.fail_times << " times";

        tvYText = oss.str();
        oss.str(""); oss.clear();

        oss << "PARALLAX: " << vins.parallax_num_view; // stringView =[NSString stringWithFormat:@"PARALLAX: %d", vins.parallax_num_view];
//        env->CallVoidMethod(tvZ, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_Z_label setText:stringView];
        tvZText = oss.str();
        oss.str(""); oss.clear();
        
        oss << "Initializing: "<< vins.initProgress; // stringView =[NSString stringWithFormat:@"Initializing: %d%%", vins.initProgress];
//        env->CallVoidMethod(tvFeature, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_feature_label2 setText:stringView];
        tvTotalText = oss.str();
        oss.str(""); oss.clear();
    
        // [_feature_label2 setHidden:NO];
        // [_feature_label3 setHidden:NO];
        // [indicator setHidden:NO];
        // [featureImageView setHidden:NO];
        initImageVisible = true;
    }   
    else {   
        if (!finish_init) {
            initImageVisible = false;
            
            start_show = true;
            finish_init = true;
        }
        
        float x_view = (float)vins.correct_Ps[frame_cnt][0];
        float y_view = (float)vins.correct_Ps[frame_cnt][1];
        float z_view = (float)vins.correct_Ps[frame_cnt][2];
        if (x_view_last == -5000)
        {
            x_view_last = x_view;
            y_view_last = y_view;
            z_view_last = z_view;
        }
        total_odom += sqrt(pow((x_view - x_view_last), 2) +
        pow((y_view - y_view_last), 2) +
        pow((z_view - z_view_last), 2));
        x_view_last = x_view;
        y_view_last = y_view;
        z_view_last = z_view;
    
        oss << "X: "<< x_view; // stringView =[NSString stringWithFormat:@"X:%.2f", x_view];
//        env->CallVoidMethod(tvX, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_X_label setText:stringView];
        tvXText = oss.str();
        oss.str(""); oss.clear();
    
        oss << "TOTAL: "<< total_odom; // stringView =[NSString stringWithFormat:@"TOTAL:%.2f", total_odom];
        // stringView = [NSString stringWithFormat:@"COST:%.2lf",vins.final_cost];
        // stringView = [NSString stringWithFormat:@"COST: %d, %.2lf",vins.visual_factor_num, vins.visual_cost];
//        env->CallVoidMethod(tvTotal, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_total_odom_label setText:stringView];
        tvTotalText = oss.str();
        oss.str(""); oss.clear();
    
        oss << "Y: "<< y_view; // stringView =[NSString stringWithFormat:@"Y:%.2f", y_view];
//        env->CallVoidMethod(tvY, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_Y_label setText:stringView];
        tvYText = oss.str();
        oss.str(""); oss.clear();

        oss << "Z: "<< z_view; // stringView =[NSString stringWithFormat:@"Z:%.2f", z_view];
//        env->CallVoidMethod(tvZ, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_Z_label setText:stringView];
        tvZText = oss.str();
        oss.str(""); oss.clear();
    }
    oss << "BUF: "<< waiting_lists; // stringView =[NSString stringWithFormat:@"BUF:%d", waiting_lists];
//    env->CallVoidMethod(tvBuf, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_buf_label setText:stringView];
    tvBufText = oss.str();
    oss.str(""); oss.clear();

    // NSString *stringZ = [NSString stringWithFormat:@"Z:%.2f",z_view, vins.f_manager.getFeatureCount()];
    if (loop_old_index != -1)
    {
        oss << "LOOP with: "<< loop_old_index; // stringView =[NSString stringWithFormat:@"LOOP with %d", loop_old_index];
//        env->CallVoidMethod(tvLoop, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_loop_label setText:stringView];
        tvLoopText = oss.str();
        oss.str(""); oss.clear();
    }
    oss << "FEATURE: "<< vins.feature_num; // stringView =[NSString stringWithFormat:@"FEATURE: %d", vins.feature_num];
//    env->CallVoidMethod(tvFeature, setTextMethod, env->NewStringUTF(oss.str().c_str())); // [_feature_label setText:stringView];
    tvFeatureText = oss.str();
    oss.str(""); oss.clear();
    
    viewUpdateMutex.unlock();
}

NSTimeInterval ViewController::get_latest_imu_time() const { return latest_imu_time; }

void ViewController::switchUI(bool isChecked) {
    if(isChecked) {
        printf("show AR\n");
        ui_main = true;
        box_in_AR = true;
        USE_PNP = true;
        image_cache_enabled = cameraMode && !USE_PNP;
    } else {
        ui_main = false;
        if (box_in_AR)
            box_in_trajectory = true;
        USE_PNP = false;
        image_cache_enabled = cameraMode && !USE_PNP;
        printf("show VINS\n");
    }
}

vector<Vector3f> ViewController::handleTap(float x, float y) {
    if (ui_main) {
        vins.drawresult.locationTapX = x;
        vins.drawresult.locationTapY = y;

        vins.drawresult.tapFlag = true;

        return vins.correct_point_cloud;
    }
    return {};
}

void ViewController::loopButtonPressed(bool isChecked) {
    if(isChecked)
    {
        DETECT_LOOP_CLOSURES = true;
        LOGI("Loop Closure enabled");
    }
    else
    {
        DETECT_LOOP_CLOSURES = false;
        LOGI("Loop Closure disabled");
    }
}

