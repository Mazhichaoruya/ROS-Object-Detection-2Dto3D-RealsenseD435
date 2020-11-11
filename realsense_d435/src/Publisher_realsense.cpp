#include "ros/ros.h"
#include "include.h"
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "realsense_d435/objection.h"
#include "realsense_d435/objectionsofonemat.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "realsense2_camera/Extrinsics.h"
#include <chrono>
#include "cuda_runtime_api.h"
#include "logging.h"
#include "common.hpp"
#include "time.h"
//#include <image_transport/image_transport.h> // 用来发布和订阅图像信息
using namespace std;
Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
Eigen::Vector3f V_T;//平移向量T
Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参
cv::Mat Depthmate,Dec_mat,color_mat;
vector<string> classNamesVec;
const auto window_name= "RGB Image";
char* input="../input/VOC";
char* output="../output/";
std::string filename;
std::string weight="../engine/";
std::string classname_path="../engine/coco.names";
//stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;
///////////////////
static float Data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
//for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
//    data[i] = 1.0;
static float prob[BATCH_SIZE * OUTPUT_SIZE];
IRuntime* runtime;
ICudaEngine* engine;
IExecutionContext* context;
int inputIndex;
int outputIndex;
void* buffers[2];
cudaStream_t stream;
vector<Objection> ObjectionOfOneMat;//一幅图中的目标
/////////////////////////////
//DNN opencv与CV-bridge冲突暂时不可用
/*Net net;//DNN net
using namespace cv::dnn;
const size_t inWidth      = 416;
const size_t inHeight     = 416;
const float WHRatio       = inWidth / (float)inHeight;
const float inScaleFactor = 1/255.f;
const float meanVal       = 127.5;
std::vector<String> outNames;
String yolo_tiny_model ="/home/mzc/code/CLionProjects/DNN435/engine/enetb0-coco_final.weights";//yolov4.weights enetb0-coco_final.weights
String yolo_tiny_cfg =  "/home/mzc/code/CLionProjects/DNN435/engine/enet-coco.cfg";//yolov4.cfg enet-coco.cfg
 */
//////////////////////////////////////////////
class Camera{
public:
    bool Colorinfo= false;
    bool Depthinfo= false;
    bool Convertinfo= false;
    std_msgs::Header this_head;
    ros::NodeHandle n;
    ros::Subscriber subrgbcaminfo;
    ros::Subscriber subdepthcaminfo;
    ros::Subscriber subrgbimg;
    ros::Subscriber subdepthimg ;
    ros::Subscriber subdepthtoclolor ;
    ros::Publisher Object_pub;
    ros::Publisher Objectimg_pub;
    Camera(){
        subrgbcaminfo= n.subscribe("/camera/color/camera_info", 1, &Camera::inforgbcallback, this);
        subdepthcaminfo = n.subscribe("/camera/depth/camera_info", 1, &Camera::infodepthcallback,this);
        subrgbimg= n.subscribe("/camera/color/image_raw", 1, &Camera::imgrgbcallback,this);
        subdepthimg = n.subscribe("/camera/depth/image_rect_raw", 1, &Camera::imgdepthcallback,this);
        subdepthtoclolor = n.subscribe( "/camera/extrinsics/depth_to_color",1, &Camera::depth_to_colorcallback,this);
        Object_pub =n.advertise<realsense_d435::objectionsofonemat>("Objects",10);
        Objectimg_pub=n.advertise<sensor_msgs::Image>("Objectsimg",1);
    }
    void inforgbcallback(const sensor_msgs::CameraInfo &caminfo){
        if (Colorinfo)
            return;
        else{
            ///获取彩色相机内参
            std::cout<<"\ncolor intrinsics: "<<endl;
            InnerTransformation_Color<<caminfo.K.at(0),caminfo.K.at(1),caminfo.K.at(2),caminfo.K.at(3),caminfo.K.at(4),caminfo.K.at(5),caminfo.K.at(6),caminfo.K.at(7),caminfo.K.at(8);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    std::cout<<InnerTransformation_Color(i,j)<<"\t";
                }
                std::cout<<std::endl;
            }
            Colorinfo= true;
        }
    }
    void infodepthcallback(const sensor_msgs::CameraInfo &caminfo){
        if (Depthinfo)
            return;
        else{
            ///获取深度相机内参
            std::cout<<"\ndepth intrinsics: "<<endl;
            Inner_Transformation_Depth<<caminfo.K.at(0),caminfo.K.at(1),caminfo.K.at(2),caminfo.K.at(3),caminfo.K.at(4),caminfo.K.at(5),caminfo.K.at(6),caminfo.K.at(7),caminfo.K.at(8);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    std::cout<<Inner_Transformation_Depth(i,j)<<"\t";
                }
                std::cout<<std::endl;
            }
            Depthinfo= true;
        }
    }
    void imgrgbcallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        color_mat=cv_ptr->image;
//        cv::imshow(window_name, color_mat);
//        cv::waitKey(30);
//        Mat color(msg->height,msg->width,CV_8UC3,cv_ptr->image.data,Mat::AUTO_STEP);
        /////////////////////////////////////
        ObjectionOfOneMat.clear();//清空上一幅图像的目标
        auto Timeofimg=this_head.stamp.toSec()-msg->header.stamp.toSec();
        if (std::abs(Timeofimg)<=0.01)//时间戳同步
            Dec_mat = Dection(color_mat);
        sensor_msgs::ImagePtr img=cv_bridge::CvImage(std_msgs::Header(),"bgr8",Dec_mat).toImageMsg();
        img->header.stamp=ros::Time::now();
        img->header.frame_id="images";
        Objectimg_pub.publish(img);
        realsense_d435::objectionsofonemat Objections;
        realsense_d435::objection objection_new;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for (auto objection:ObjectionOfOneMat) {
            if (objection.Enable) {
                pcl::PointXYZRGB p;
                objection_new.classname=objection.Classname;
                objection_new.classID=objection.ClassID;
                p.r=255-objection.ClassID*3;
                p.g=objection.ClassID*3;
                p.b=objection.ClassID*2;
                objection_new.center_point.x=objection.Point_Camera.at(0);
                objection_new.center_point.y=objection.Point_Camera.at(1);
                objection_new.center_point.z=objection.Point_Camera.at(2);
                for (auto i:objection.Real_Point) {
                    geometry_msgs::Point32 point;
                    point.x=i[0]/1000.0;
                    point.y=i[1]/1000.0;
                    point.z=i[2]/1000.0;
                    p.x=point.x;
                    p.y=point.y;
                    p.z=point.z;
                    objection_new.point_cloud.push_back(point);
                    cloud.points.push_back(p);
                }
            }
            Objections.objectionsofonemat.push_back(objection_new);
        }
//        sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", Dec_mat).toImageMsg();
//        Objection_img.publish(msg);
        pcl::toROSMsg(cloud,Objections.pointcloud);
        Objections.pointcloud.header.stamp=ros::Time::now();
        Objections.pointcloud.header.frame_id="objects";
        Objections.sizeofobjections=ObjectionOfOneMat.size();
        Object_pub.publish(Objections);//发布信息--一幅图像的信息
//        if (cv::waitKey(1) != 27)
//            imshow(window_name, Dec_mat);
        Objections.objectionsofonemat.clear();
        /////////////////////////////////
    }
    void imgdepthcallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        Depthmate=cv_ptr->image;
        this_head=msg->header;
//        cout<<Depthmate.at<uint16_t>(240,320)<<endl;
//     cv::imshow("depth", Depthmate);
//     cv::waitKey(30);

    }
    void depth_to_colorcallback(const realsense2_camera::Extrinsics &extrin) {
        if (Convertinfo)
            return;
        else{
            ///获取深度相机相对于彩色相机的外参，即变换矩阵: P_color = R * P_depth + T
            std::cout<<"\nextrinsics of depth camera to color camera: \nrotaion: "<<std::endl;
            MTR<<extrin.rotation[0],extrin.rotation[1],extrin.rotation[2],extrin.rotation[3],extrin.rotation[4],extrin.rotation[5],extrin.rotation[6],extrin.rotation[7],extrin.rotation[8];
            for(int i = 0; i < 3; ++i){
                for(int j = 0; j < 3; ++j){
                    std::cout<<MTR(i,j)<<"\t";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl;
            std::cout<<"translation: ";
            V_T<<extrin.translation[0],extrin.translation[1],extrin.translation[2];
            for(int i=0;i<3;i++)
                std::cout<<V_T(i)<<"\t";
            std::cout<<std::endl;
            Convertinfo= true;
        }
    }
    void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context.enqueue(batchSize, buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
    }
    cv::Mat Dection(cv::Mat img){
        int fcount = 0;
        fcount++;
        for (int b = 0; b < fcount; b++) {
            if (img.empty()) continue;
            cv::Mat pr_img = preprocess_img(img); // letterbox BGR to RGB
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    Data[b * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
                    Data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    Data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }
        // Run inference
        auto start = std::chrono::system_clock::now();
        doInference(*context, stream, buffers, Data, prob, BATCH_SIZE);
        auto end = std::chrono::system_clock::now();
//        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            std::cout <<"num:"<<res.size()<<" ";
            for (size_t j = 0; j < res.size(); j++) {
                cv::Rect r = get_rect(img, res[j].bbox);
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img, classNamesVec[(int)res[j].class_id], cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                std::cout<<classNamesVec[(int)res[j].class_id]<<";";
                if (VIDEO_TYPE==3)
                    ObjectionOfOneMat.push_back(Objection(r,(int)res[j].class_id));
            }
            std::cout<<std::endl;
            if (VIDEO_TYPE==1)
                cv::imwrite(std::string(output) + filename, img);
        }
        return img;
    }
};



int main(int argc, char** argv)
{
//    image_detection_Cfg();//DNN二维目标检测初始化
//////////////TensorRT//////////////////
    cudaSetDevice(DEVICE);
    std::vector<std::string> file_names;
    clock_t last_time;
    // create a model using the API directly and serialize it to a stream
    char *trtModelStream{ nullptr };
    size_t size{ 0 };
    std::string engine_name = STR2(NET);
    engine_name = weight+"yolov5" + engine_name + ".engine";
    std::ifstream file(engine_name);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    std::ifstream classNamesFile(classname_path);
    if (classNamesFile.is_open())
    {
        std::string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
    // prepare input data ---------------------------
    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr)  ;
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
    // Create stream
    CHECK(cudaStreamCreate(&stream));
    //////////////TensorRT载入模型////////////////////
    ros::init(argc,argv,"realsense_Objection");
    ///////////////
    Camera CA;
    ////////////////////////
    ros::spin();
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    return 0;

}
//////////////////////////Opencv 版本冲突 暂时无法使用DNN
/*
void image_detection_Cfg() {
    net = readNetFromDarknet(yolo_tiny_cfg, yolo_tiny_model);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);// DNN_BACKEND_INFERENCE_ENGINE DNN_BACKEND_OPENCV 未安装IE库 if you have IntelCore CPU you can chose this Para to accelerate youe model--Openvino;
    net.setPreferableTarget(DNN_TARGET_CPU);
    outNames = net.getUnconnectedOutLayersNames();
    for (int i = 0; i < outNames.size(); i++) {
        printf("output layer name : %s\n", outNames[i].c_str());
    }
    ifstream classNamesFile(classname_path);
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
}
Mat Dectection(Mat &color_mat) {  //getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0
    // 加载图像 D435加载
//        cout<<color_mat.type()<<endl;
    Mat inputBlob = blobFromImage(color_mat, inScaleFactor, Size(inWidth, inHeight), Scalar(), true, false);
    net.setInput(inputBlob);
    // 检测
    std::vector<Mat> outs;
    net.forward(outs, outNames);
    vector<double> layersTimings;
    double freq = getTickFrequency() / 1000;
    double time = net.getPerfProfile(layersTimings) / freq;
    double FPS=1000/time;
    ostringstream ss;
    ss << "FPS: " << FPS ;
    putText(color_mat, ss.str(), Point(0, 10), 0, 0.5, Scalar(255, 0, 0));
    vector<Rect> boxes;
    vector<int> classIds;
    vector<float> confidences;
    for (size_t i = 0; i < outs.size(); ++i) {
        float *data = (float *) outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > 0.5) {
                int centerX = (int) (data[0] * color_mat.cols);
                int centerY = (int) (data[1] * color_mat.rows);
                int width = (int) (data[2] * color_mat.cols);
                int height = (int) (data[3] * color_mat.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float) confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    vector<int> indices;
    NMSBoxes(boxes, confidences, 0.5, 0.2, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        Rect box = boxes[idx];
        auto ClassID=classIds[idx];
        String className = classNamesVec[classIds[idx]];
        Objection NewObjection(box,ClassID);
        ObjectionOfOneMat.push_back(NewObjection);
        putText(color_mat, className.c_str(), box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);
        rectangle(color_mat, box, Scalar(0, 0, 255), 2, 8, 0);
    }
    return color_mat;
}
*/
/////////////////////////////////////////////////
