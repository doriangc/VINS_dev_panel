#ifndef VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H
#define VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H



/*
 * This is the class which corresponds to the ViewController-Objective C Class.
 * I started by copying all the code over 
 * and moved on by tweaking all the compile-errors
 * In that process i never deleted the original code but commented it out where i had replaced it
 * this was done with the intention of being able to quickly lock back into the original code
 */

#include <iostream>

#include "utility.hpp"
// iOS Specific OpenCV Parts
// #import "CameraUtils.h" (another Objective C header)
// #import <opencv2/imgcodecs/ios.h>
// #import <opencv2/videoio/cap_ios.h>
#include "feature_tracker.hpp"

#include "global_param.hpp"
#include "VINS.hpp"
#include <queue>
#include "draw_result.hpp"

#include "keyframe.h"
#include "loop_closure.h"
#include "keyfame_database.h"
#include <sys/utsname.h>

// added in the continous process of tranlating objective c code
#include <condition_variable> // std::condition_variable con
// #include <android/log.h>
#define LOG_TAG "ViewController.cpp"
// #define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
// #define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
// #define printf(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

typedef double NSTimeInterval;
#define APPNAME "VINS_Android"
#include <thread>
// #include <jni.h>

#define LOGI(...) printf(__VA_ARGS__); printf("\n")
#define LOGE(...) printf(__VA_ARGS__); printf("\n")
#define __android_log_print(prio, tag, fmt, ...) printf(fmt, ##__VA_ARGS__)



extern "C" {
    #include <time.h>
}

// #include <android/looper.h>
// #include <android/sensor.h>

struct IMUMessage {
    NSTimeInterval header;
    Vector3d acc;
    Vector3d gyr;
};

// Image metadata
// Includes the timestamp as well as any point clouds associated
// with this image
struct ImageMeta {
    NSTimeInterval timestamp;
    map<int, Vector3d> point_clouds;
};

struct ImageData {
    NSTimeInterval header;
    cv::Mat image; // UIImage *image;
};

struct ImageDataCache {
    NSTimeInterval header;
    cv::Mat equ_image;
    cv::Mat image; // UIImage *image;
};

struct VinsDataCache {
    NSTimeInterval header;
    Vector3f P;
    Matrix3f R;
};

typedef shared_ptr <IMUMessage const > ImuConstPtr;
typedef shared_ptr <ImageMeta const > ImgConstPtr;
//@end


class ViewController {

private:
    const int videoWidth = 480;
    const int videoHeight = 640;

    bool isCapturing;
    cv::Ptr<FeatureTracker> feature_tracker;
    cv::Size frameSize;
    std::mutex _condition;          // NSCondition *_condition;
    std::thread mainLoop;          // NSThread *mainLoop;
    std::thread draw;              // NSThread *draw;
    std::thread saveData;          // NSThread *saveData;
    std::thread loop_thread;       // NSThread *loop_thread;
    std::thread globalLoopThread;  // NSThread *globalLoopThread;

/*************************** Save data for debug ***************************/

    bool start_record = false;

    bool start_playback = false;

    bool start_playback_vins = false;

    unsigned long imageDataIndex = 0;

    unsigned long imageDataReadIndex = 0;

    unsigned long imuDataIndex = 0;

    unsigned long imuDataReadIndex = 0;

    unsigned long vinsDataIndex = 0;

    unsigned long vinsDataReadIndex = 0;

    queue<ImageData> imgDataBuf;

    ImageData imgData;

    IMUMessage imuData;

    KEYFRAME_DATA vinsData;

/*************************** Save data for debug ***************************/

/******************************* UI CONFIG *******************************/

    // false: VINS trajectory is the main view, AR image is in left bottom
    // true: AR image is the main view, VINS is in left bottom
    bool ui_main = false;

    bool box_in_AR = false;

    bool box_in_trajectory = false;

    // If initialized finished, start show is true
    bool start_show = false;

    int loop_old_index = -1;

    float x_view_last = -5000;

    float y_view_last = -5000;

    float z_view_last = -5000;

    float total_odom = 0;

/******************************* UI CONFIG *******************************/

    FeatureTracker featuretracker;

    VINS vins;

    // Store the feature data processed by featureTracker
    queue<ImgConstPtr> img_msg_buf;

    // Store the IMU data for vins
    queue<ImuConstPtr> imu_msg_buf;

    // Store the IMU data for motion-only vins
    // queue<IMUMsgLocal> local_imu_msg_buf;

    // The number of measurements waiting to be processed
    int waiting_lists = 0;

    int frame_cnt = 0;

    // Lock the feature and imu data buffer
    std::mutex m_feature_and_IMU_buffer_mutex;

    std::condition_variable con;

    NSTimeInterval current_time = -1;

    NSTimeInterval latest_imu_time = -1;

    std::string saveDir = "";
public:
    NSTimeInterval get_latest_imu_time() const;

private:

    int imu_prepare = 0;

// MotionManager for read imu data
    // IMU Things:
    const int LOOPER_ID_USER = 3;
    const int SENSOR_REFRESH_RATE_HZ = 100;
    const int32_t SENSOR_REFRESH_PERIOD_US = int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);

    // static ASensorEventQueue *accelerometerEventQueue;
    // static ASensorEventQueue *gyroscopeEventQueue;

    static int process_imu_looper_events(int fd, int events, void* data);
    static int process_imu_sensor_events(int fd, int events, void* data);

    // singleton for static callbacks
    static ViewController* instance;

    // Segment the trajectory using color when re-initialize
    int segmentation_index = 0;

    // Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
    // Set false: 10 HZ pose output and AR rendering in back-end
    bool USE_PNP = true;

    // Lock the solved VINS data feedback to featuretracker
    std::mutex m_depth_feedback_mutex;

    // Lock the IMU data feedback to featuretracker
    std::mutex m_imu_feedback_mutex;

    // Solved VINS feature feedback to featuretracker
    list<IMG_MSG_LOCAL> solved_features;

    // Solved VINS status feedback to featuretracker
    VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

    // Raw image data buffer for extracting FAST feature
    queue<pair<cv::Mat, double>> image_buf_loop;

    // Lock the image_buf_loop
    std::mutex m_image_buf_loop;

    // Detect loop
    LoopClosure *loop_closure = nullptr;

    // Keyframe database
    KeyFrameDatabase keyframe_database;

    // Control the loop detection frequency
    int keyframe_freq = 0;

    // Index the keyframe
    int global_frame_cnt = 0;

    // Record the checked loop frame
    int loop_check_cnt = 0;

    // Indicate if brief vocabulary read finish
    bool voc_init_ok = false;

    // Indicate the loop frame index
    int old_index = -1;

    // Translation drift
    Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);

    // Rotation drift
    Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

/******************************* Loop Closure ******************************/

    // MARK: Unity Camera Mode Switching
    // Ground truth from UI switch property "self.switchUIAREnabled"

    // Implied, updated by updateCameraMode()
    bool imuPredictEnabled = false;

    // Implied, updated by updateCameraMode()
    bool cameraMode = true;

    // Implied, updated by updateCameraMode()
    bool image_cache_enabled = cameraMode && !USE_PNP;
    
public:
    ViewController();
    ~ViewController();

    inline static double timeStampToSec (long timeStamp) { return timeStamp / 1000000000.0; };
    static NSTimeInterval systemUptime();

    std::mutex viewUpdateMutex;
    std::string tvXText;
    std::string tvYText;
    std::string tvZText;
    std::string tvFeatureText;
    std::string tvTotalText{"TOTAL:"};
    std::string tvBufText;
    std::string tvLoopText{"LOOP:"};
    bool initImageVisible = true;
    
    float virtualCamDistance = 5;


    void viewDidLoad();

/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
    queue<ImageDataCache> image_pool;
    queue<VinsDataCache> vins_pool;
    ImageDataCache image_data_cache;
    cv::Mat lateast_equa;
    cv::Mat latest_image; //UIImage *latest_image;
    Vector3f latest_P;
    Matrix3f latest_R;

    cv::Mat pnp_image;
    Vector3d pnp_P;
    Matrix3d pnp_R;

    /**
     * Takes RGBA doesnt change the format!
     */
    void processImage(cv::Mat& image, double timestamp, bool isScreenRotated);


/*
 Send imu data and visual data into VINS
 */
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();


    /**
     * Used for feature tracking, returns and removes all imu_msg from the local imu buffer
     * that are in between the last and the current header (timeStamp)
     */
    // vector<IMUMsgLocal> getImuMeasurements(double header);

    void send_imu(const ImuConstPtr &imu_msg);

    //TODO: solve quick fix
    bool mainLoop_isCancelled = false;
    /**
     * VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
     * If the newest frame is keyframe, then push it into keyframe database
     */
    void run();

    int kf_global_index;
    bool start_global_optimization = false;
    void process();


    //TODO: solve quick fix
    bool loop_thread_isCancelled = false;
    /**
     * Loop detection thread: this thread detect loop for newest keyframe and retrieve features
     */
    void loopDetectionLoop();

    //TODO: solve quick fix
    bool globalLoopThread_isCancelled = false;
    /**
     * GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
     */
    void globalPoseGraphLoop();

    /*
     * Z^
     * |   /Y
     * |  /
     * | /
     * |/
     * --------->X
     * IMU data process and interpolation 
     */
    bool imuDataFinished = false;
    bool vinsDataFinished = false;
    shared_ptr<IMUMessage> cur_acc = shared_ptr<IMUMessage>(new IMUMessage()); // shared_ptr<IMUMessage> cur_acc(new IMUMessage());
    vector<IMUMessage> gyro_buf;  // for Interpolation
    void imuStartUpdate() const;
    static void imuStopUpdate();

//    static char *briefFilePath;
//
//    void setBriefFile(char *filePath);

/********************************************************************UI View Controler********************************************************************/
    void showInputView();

    // Where is this coming from (delegate)? It's not being called anywhere.
    void showOutputImage(cv::Mat* image) {
//-(void)showOutputImage:(UIImage*)image
//{
        //TODO: [featureImageView setImage:image];
    }
/********************************************************************UI View Controler********************************************************************/


/********************************************************************UI Button Controler********************************************************************/


    // TODO: UI Interaction
    /// switch UI between graph(VINS) and camera(AR)
    void switchUI(bool isChecked);
    /*
    void switchUI(int selectedSegmentIndex) {
//-(IBAction)switchUI:(UISegmentedControl *)sender
//{
        switch(selectedSegmentIndex) //switch (_switchUI.selectedSegmentIndex)
        {
            case 0:
                switchUIAREnabled = true; // self.switchUIAREnabled = YES;

                printf("show AR\n");
                ui_main = true;
                box_in_AR= true;
                USE_PNP = true;
                image_cache_enabled = cameraMode && !USE_PNP;
                break;
            case 1:
                switchUIAREnabled = false; // self.switchUIAREnabled = NO;

                ui_main = false;
                if (box_in_AR)
                    box_in_trajectory = true;
                USE_PNP = false;
                image_cache_enabled = cameraMode && !USE_PNP;
                printf("show VINS\n");
                break;
            default:
                break;
        }
    }
    */

    /* Does nothing (there is no fov Slider)
    IBAction fovSliderValueChanged(id sender) {
//- (IBAction)fovSliderValueChanged:(id)sender {
        self.fovLabel.text = [[NSNumber numberWithFloat:self.fovSlider.value] stringValue];
    }
    */
    
    // TODO: Gestensteuerung?
    // pan
    /*
    void handlePan(UIPanGestureRecognizer* recognizer) {
//- (void) handlePan:(UIPanGestureRecognizer*) recognizer
//{
        if(ui_main and 0)
            return;

        if (!ui_main)
        {
            CGPoint translation = [recognizer translationInView:self.view];
            CGFloat velocityX = [recognizer velocityInView:self.view].x;
            CGFloat velocityY = [recognizer velocityInView:self.view].y;
            //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
            static CGFloat vx_last = 0;
            static CGFloat vy_last = 0;

            CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
            CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
            vx_last = vx_smooth;
            vy_last = vy_smooth;
            if(recognizer.numberOfTouches == 2)
            {
                vins.drawresult.Y0 += vx_smooth/100.0;
                vins.drawresult.X0 += vy_smooth/100.0;
            }
            else // Rotate View
            {
                vins.drawresult.theta += vy_smooth/100.0;
                vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
                vins.drawresult.phy += vx_smooth/100.0;
                vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
            }

            vins.drawresult.change_view_manualy = true;
        }
        else
        {
            CGPoint translation = [recognizer translationInView:self.view];
            CGFloat velocityX = [recognizer velocityInView:self.view].x;
            CGFloat velocityY = [recognizer velocityInView:self.view].y;
            //CGFloat translationX =
            //CGFloat translationY = [recognizer translationInView:self.view].y;
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "!!!!!!!!!!!!!!!!!!!!!!%f  %f", imageView.frame.size.height, imageView.frame.size.width);
            CGPoint point = [recognizer locationInView:self.view];
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "X Location: %f", point.x);
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "Y Location: %f",point.y);

            //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
            static CGFloat vx_lastAR = 0;
            static CGFloat vy_lastAR = 0;

            CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
            CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
            vx_lastAR = vx_smooth;
            vy_lastAR = vy_smooth;
            if(recognizer.numberOfTouches == 2)
            {

                vins.drawresult.Y0AR += vx_smooth/100.0;
                vins.drawresult.X0AR += vy_smooth/100.0;

                vins.drawresult.locationXT2 = point.x * 640.0 / imageView.frame.size.width;
                vins.drawresult.locationYT2 = point.y * 480.0 / imageView.frame.size.height;

                vins.drawresult.finger_s = 0;
                vins.drawresult.finger_p = 0;
                if ((vins.drawresult.finger_d ++) > 7)
                    vins.drawresult.finger_state = 2;
            }
            else
            {
                vins.drawresult.thetaAR += vy_smooth/100.0;
                //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
                vins.drawresult.phyAR += vx_smooth/100.0;
                //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);

                vins.drawresult.locationX = point.x * 640.0 / imageView.frame.size.width;
                vins.drawresult.locationY = point.y * 480.0 / imageView.frame.size.height;

                vins.drawresult.finger_d = 0;
                vins.drawresult.finger_p = 0;
                if ((vins.drawresult.finger_s ++) > 7)
                    vins.drawresult.finger_state = 1;
            }
        }


    }
    */

    // pinch
    /*
    void handlePinch(UIPinchGestureRecognizer* recognizer) {
//- (void) handlePinch:(UIPinchGestureRecognizer*) recognizer
//{
        if(ui_main and 0)
            return;

        if (!ui_main)
        {
            vins.drawresult.change_view_manualy = true;
            if(vins.drawresult.radius > 5 || recognizer.velocity < 0)
                vins.drawresult.radius -= recognizer.velocity * 0.5;
            else
            {
                vins.drawresult.Fx += recognizer.velocity * 15;
                if(vins.drawresult.Fx < 50)
                    vins.drawresult.Fx = 50;
                vins.drawresult.Fy += recognizer.velocity * 15;
                if(vins.drawresult.Fy < 50)
                    vins.drawresult.Fy = 50;
            }
        }
        else{

            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_d = 0;
            if ((vins.drawresult.finger_p ++) > 7)
                vins.drawresult.finger_state = 3;

            CGPoint point = [recognizer locationInView:self.view];
            vins.drawresult.locationXP = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationYP = point.y * 480.0 / imageView.frame.size.height;

            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "pipikk_radius: %f velocity: ", vins.drawresult.radiusAR, recognizer.velocity);

            //if(vins.drawresult.radiusAR > 5 || recognizer.velocity < 0)
            //{
            vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
            //}
        }
    }
    */
    
    // tap
    /*
    void handleTap(UITapGestureRecognizer* recognizer) {
//- (void) handleTap:(UITapGestureRecognizer*) recognizer
//{
        if (!ui_main)
        {

        }
        else{

            // vins.drawresult.finger_s = 0;
            // vins.drawresult.finger_d = 0;
            // if ((vins.drawresult.finger_p ++) > 7)
            // vins.drawresult.finger_state = 3;

            CGPoint point = [recognizer locationInView:self.view];
            vins.drawresult.locationTapX = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationTapY = point.y * 480.0 / imageView.frame.size.height;

            vins.drawresult.tapFlag = true;

        }
    }
    */

    // long press
    /*
    void handleLongPress(UILongPressGestureRecognizer* recognizer) {
//- (void) handleLongPress:(UILongPressGestureRecognizer*) recognizer
//{
        if (!ui_main)
        {

        }
        {
            CGPoint point = [recognizer locationInView:self.view];
            vins.drawresult.locationLongPressX = point.x * 640.0 / imageView.frame.size.width;
            vins.drawresult.locationLongPressY = point.y * 480.0 / imageView.frame.size.height;
            vins.drawresult.longPressFlag = true;
        }
    }
    */

    vector<Vector3f> handleTap(float x, float y);

    
    /// loop button
    static void loopButtonPressed(bool isChecked);
    /*
    IBAction loopButtonPressed(id sender) {
//- (IBAction)loopButtonPressed:(id)sender {
        if(DETECT_LOOP_CLOSURES)
        {
            DETECT_LOOP_CLOSURES = false;
            [_loopButton setTitle:@"ENLOOP" forState:UIControlStateNormal];
        }
        else
        {
            DETECT_LOOP_CLOSURES = true;
            [_loopButton setTitle:@"UNLOOP" forState:UIControlStateNormal];
        }
        
        // start_record = !start_record;
        // if(start_record)
        // {
        // start_playback = false;
        // [_recordButton setTitle:@"Stop" forState:UIControlStateNormal];
        // [saveData start];
        // }
        // else
        // {
        // TS(record_imu);
        // imuData.header = 0; // as the ending marker
        // imuData.acc << 0,0,0;
        // imuData.gyr << 0,0,0;
        // [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
        // [self recordImu];
        // TE(record_imu);
        // [_recordButton setTitle:@"Record" forState:UIControlStateNormal];
        // }   
    }
    */

    // reinit button
    /*
    IBAction reinitButtonPressed(id sender) {
//- (IBAction)reinitButtonPressed:(id)sender {
        vins.drawresult.planeInit = false;
        vins.failure_hand = true;
        vins.drawresult.change_color = true;
        vins.drawresult.indexs.push_back(vins.drawresult.pose.size());
        segmentation_index++;
        keyframe_database.max_seg_index++;
        keyframe_database.cur_seg_index = keyframe_database.max_seg_index;
        
        // start_playback = !start_playback;
        // if(start_playback)
        // {
        // //TS(read_imu);
        // NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        // NSString *documentsPath = [paths objectAtIndex:0];
        // NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
        // imuReader = [NSData dataWithContentsOfFile:filePath];
        // //TE(read_imu);
        // start_record = false;
        // [_playbackButton setTitle:@"Stop" forState:UIControlStateNormal];
        // }
        // else
        // [_playbackButton setTitle:@"Playback" forState:UIControlStateNormal];
    }
    */

/********************************************************************UI Button Controler********************************************************************/


/***********************************************************About record and playback data for debug********************************************************/

    // iOS Memory-Warning Handling:
    /* isn't actually disposing anything...
    void didReceiveMemoryWarning() {
//- (void)didReceiveMemoryWarning {
        [super didReceiveMemoryWarning];
        // Dispose of any resources that can be recreated.
    }
    */

    //TODO: solve quick fix
    bool saveData_isCancelled = false;
    void saveDataLoop();

    void tapSaveImageToIphone(cv::Mat* image) {
    //- (void)tapSaveImageToIphone:(UIImage*)image
    //{
        // TODO: save image to Album
        // UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
    }

//TODO: is this even used anywhere? -> seems to be a callback to process error while saving to filesystem
    /*
    void image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{
//- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{

            if (error == nil) {
                __android_log_print(ANDROID_LOG_INFO, APPNAME, "save access");
            }else{
                __android_log_print(ANDROID_LOG_INFO, APPNAME, "save failed");
            }
    }
    */

//TODO: do directory management work
    /*
    void checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath {
//- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
//{
        //delete already exist directory first time
        NSError *error;
        if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])	//Does directory exist?
        {
            if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])	//Delete it
            {
                __android_log_print(ANDROID_LOG_INFO, APPNAME, "Delete directory error: %@", error);
            }
        }

        //creat file directory if it does not exist
        if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
        {
            __android_log_print(ANDROID_LOG_INFO, APPNAME, "directory does not exist");
            if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
            withIntermediateDirectories:NO
            attributes:nil
            error:&error])
            {
                __android_log_print(ANDROID_LOG_INFO, APPNAME, "Create directory error: %@", error);
            }
        }
    }
    */

    void recordImu() {
    //- (void)recordImu
    //{
        // TODO: write imuDataBuf to file
        /*
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [paths objectAtIndex:0];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name

        [imuDataBuf writeToFile:filePath atomically:YES];
        //[msgData writeToFile:filePath atomically:YES];
        */
    }

    void recordVins() {
    //- (void)recordVins
    //{
        // TODO: write vinsDataBuf to file
        /*
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [paths objectAtIndex:0];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name

        [vinsDataBuf writeToFile:filePath atomically:YES];
        //[msgData writeToFile:filePath atomically:YES];
        */
    }

    void recordImageTime(ImageData& image_data) {
//- (void)recordImageTime:(ImageData&)image_data
//{
        // TODO: write image_data.header (time) to file
        /*
        double time = image_data.header;
        NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory

        [self checkDirectoryPath:imageDataIndex withObject:documentsPath];

        NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

        [msgData writeToFile:filePath atomically:YES];
        */
    }

    void recordImage(ImageData& image_data) {
//- (void)recordImage:(ImageData&)image_data
//{
        // TODO: write image_data.image to file
        /*
        NSData *msgData = UIImagePNGRepresentation(image_data.image);
        //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory

        [self checkDirectoryPath:imageDataIndex withObject:documentsPath];

        NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

        [msgData writeToFile:filePath atomically:YES];
        */
    }

    /**
     * tries to read image time from file and writes it into property imgData.header
     */
    bool readImageTime(unsigned long index) {
//-(bool)readImageTime:(unsigned long)index
//{
        return false;
        // TODO: implement
        /*
        bool file_exist;
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
        NSString *filename = [NSString stringWithFormat:@"%lu", index];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

        //check file exists
        if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
        {
            NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
            if (file1)
            {
                double time;
                [file1 getBytes:&time length:sizeof(time)];
                imgData.header = time;
            }
            file_exist = true;
        }
        else
        {
            file_exist = false;
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "File does not exist");
        }
        return file_exist;
        */
    }

    /**
     * tries to read image from file and writes it into property imgData.image
     */
    bool readImage(unsigned long index) {
//-(bool)readImage:(unsigned long)index
//{
        return false;
        // TODO: implement
        /*
        bool file_exist;
        NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
        NSString *filename = [NSString stringWithFormat:@"%lu", index];
        NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name

        //check file exists
        if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
        {
            NSData *pngData = [NSData dataWithContentsOfFile:filePath];
            imgData.image = [UIImage imageWithData:pngData];
            file_exist = true;
        }
        else
        {
            file_exist = false;
            //__android_log_print(ANDROID_LOG_INFO, APPNAME, "File does not exist");
        }
        return file_exist;
        */
    }

/**************************************************************About record and playback data for debug**********************************************************/

    /* iOS State Management
    void viewDidAppear(BOOL animated){
//- (void)viewDidAppear:(BOOL)animated
//{
        [super viewDidAppear:animated];
    }
     */

    // TODO: call when app closes
    void viewDidDisappear() {
//- (void)viewDidDisappear:(BOOL)animated
//{
        // [super viewDidDisappear:animated];
        if (isCapturing)
        {
            // TODO: stop the camera [videoCamera stop];
        }
        mainLoop_isCancelled = true; // [mainLoop cancel];
        // TODO: where does it get started? [draw cancel];
#ifdef LOOP_CLOSURE
        loop_thread_isCancelled = true; // [loop_thread cancel];
#endif
    }

    void viewDidUnload() {
//-(void)viewDidUnload{
        /*
        [motionManager stopAccelerometerUpdates];
        [motionManager stopDeviceMotionUpdates];
        [motionManager stopGyroUpdates];
        [motionManager stopMagnetometerUpdates];
        [super viewDidUnload];
         */
    }

    void dealloc() {
//- (void)dealloc
//{
//        videoCamera.delegate = nil;
    }

/*
 Check the device
 */
    static DeviceType deviceName();

    // rewrite this check for required android api level 
    bool isOSCompatible();

    void setSaveDir(std::string saveDir) {
        this->saveDir = saveDir;
    }
//@end

};


#endif //VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H