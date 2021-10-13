/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/10/8 下午7:57
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/10/8 下午7:57
 * @Version 1.0
 */

#include <yolo_trt/yolo_trt.h>

namespace YoloTensorRTNS
{
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int DETECTION_SIZE = sizeof(Yolo::Detection) / sizeof(float);
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * DETECTION_SIZE + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
    const char* INPUT_BLOB_NAME = "data";
    const char* OUTPUT_BLOB_NAME = "prob";
    static Logger gLogger;

    // prepare input data ---------------------------
    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];

    YoloTensorRT::YoloTensorRT()
    : nh_()
    , pnh_("~")
    {
        ROS_INFO("[YoloObjectDetector] Node started.");
        initROS();
        loadEngine();
        loadClassNames();
    }

    YoloTensorRT::~YoloTensorRT()
    {

    }

    void YoloTensorRT::initROS()
    {
        rgb_sub_ = nh_.subscribe("cv_camera/image_raw", 10, &YoloTensorRT::rgb_cb, this);
        bbox_pub_ = nh_.advertise<darknet_ros_msgs::BoundingBoxes>("yolo_trt/bounding_boxes", 1);
    }


    void YoloTensorRT::rgb_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            imageHeader_ = msg->header;
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("cv_bridge exception %s ", ex.what());
        }
        Mat img = cv_ptr->image;
        cv_bridge::CvImage cvImage;
        cvImage.header.stamp = ros::Time::now();
        cvImage.header.frame_id = "detection_image";
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvImage.image = img.clone();

        int fcount  = 0;
        fcount++;
        for (int b = 0; b < fcount; b++)
        {
            if(img.empty())
                continue;
            Mat pr_img = preprocess_img(img);
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row)
            {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col)
                {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }

        // Run inference
        auto start = chrono::system_clock::now();
        doInference(*context, data, prob, BATCH_SIZE);
        auto end = chrono::system_clock::now();
        // cout << chrono::duration_cast<chrono::milliseconds>(end - start).count() << "ms" << endl;
        float fps = 1.0f / chrono::duration_cast<chrono::milliseconds>(end - start).count() * 1000;
//        putText(cvImage.image, "FPS: " + to_string(fps), Point(0, 20), FONT_HERSHEY_PLAIN, 1.2, Scalar(255, 255, 255), 2);
        cout << fps << endl;
        vector<vector<Yolo::Detection>> batch_res(fcount);

        for (int b = 0; b < fcount; b++)
        {
            auto& res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE]);
        }

        for (int b = 0; b < fcount; b++)
        {
            auto& res = batch_res[b];
            // cout << res.size() << endl;
            for (size_t j = 0; j < res.size(); j++)
            {
                Rect r = get_rect(img, res[j].bbox);
                box.xmin = r.x;
                box.ymin = r.y;
                box.xmax = r.x + r.width;
                box.ymax = r.y + r.height;
                box.probability = res[j].det_confidence;
                box.Class = classNamesVec[(int)res[j].class_id];
                boundingBoxes.bounding_boxes.push_back(box);


//                rectangle(img, r, Scalar(255, 0, 255), 2);
//                putText(img, classNamesVec[(int)res[j].class_id], Point(r.x, r.y - 20), FONT_HERSHEY_PLAIN, 1.2, Scalar(255, 255, 255), 2);
//                putText(img, to_string(res[j].det_confidence), Point(r.x, r.y), FONT_HERSHEY_PLAIN, 1.2, Scalar(255, 255, 255), 2);

            }
        }
        boundingBoxes.header.stamp = ros::Time::now();
        boundingBoxes.header.frame_id = "detection";
        boundingBoxes.header = imageHeader_;
        boundingBoxes.image = *(cvImage.toImageMsg());

        bbox_pub_.publish(boundingBoxes);
        boundingBoxes.bounding_boxes.clear();

//        imshow("result", img);
//        waitKey(1);
    }

    void YoloTensorRT::loadClassNames()
    {
        string classFile = "/home/ubuntu/yolo_trt_ros/src/yolo_trt/name/voc.names";
        ifstream classNameFiles(classFile);
        if (classNameFiles.is_open())
        {
            string className = "";
            while (getline(classNameFiles, className))
                classNamesVec.push_back(className);
        }
        else
            cerr << "load class file not right";
    }

    void YoloTensorRT::loadEngine()
    {
        string engine_file = "/home/ubuntu/yolo_trt_ros/src/yolo_trt/engine/yolov4.engine";
        ifstream file(engine_file, ios::binary);

        if (file.good())
        {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else {
            cerr << "load engine not right" << endl;
        }

        runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        context = engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;
    }

    void YoloTensorRT::doInference(IExecutionContext& context, float* input, float* output, int batchSize)
    {
        const ICudaEngine& engine = context.getEngine();

        // Pointers to input and output device buffers to pass to engine.
        // Engine requires exactly IEngine::getNbBindings() number of buffers.
        assert(engine.getNbBindings() == 2);
        void* buffers[2];

        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);

        // Create GPU buffers on device
        CUDA_CHECK(cudaMalloc(&buffers[inputIndex], batchSize * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float)));

        // Create stream
        cudaStream_t stream;
        CUDA_CHECK(cudaStreamCreate(&stream));

        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CUDA_CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context.enqueue(batchSize, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);

        // Release stream and buffers
        cudaStreamDestroy(stream);
        CUDA_CHECK(cudaFree(buffers[inputIndex]));
        CUDA_CHECK(cudaFree(buffers[outputIndex]));
    }

    Mat YoloTensorRT::preprocess_img(Mat &img)
    {
        int w, h, x, y;
        float r_w = INPUT_W / (img.cols*1.0);
        float r_h = INPUT_H / (img.rows*1.0);
        if (r_h > r_w) {
            w = INPUT_W;
            h = r_w * img.rows;
            x = 0;
            y = (INPUT_H - h) / 2;
        } else {
            w = r_h* img.cols;
            h = INPUT_H;
            x = (INPUT_W - w) / 2;
            y = 0;
        }
        Mat re(h, w, CV_8UC3);
        resize(img, re, re.size());
        Mat out(INPUT_H, INPUT_W, CV_8UC3, Scalar(128, 128, 128));
        re.copyTo(out(Rect(x, y, re.cols, re.rows)));
        return out;
    }

    Rect YoloTensorRT::get_rect(Mat &img, float *bbox)
    {
        int l, r, t, b;
        float r_w = INPUT_W / (img.cols * 1.0);
        float r_h = INPUT_H / (img.rows * 1.0);
        if (r_h > r_w) {
            l = bbox[0] - bbox[2]/2.f;
            r = bbox[0] + bbox[2]/2.f;
            t = bbox[1] - bbox[3]/2.f - (INPUT_H - r_w * img.rows) / 2;
            b = bbox[1] + bbox[3]/2.f - (INPUT_H - r_w * img.rows) / 2;
            l = l / r_w;
            r = r / r_w;
            t = t / r_w;
            b = b / r_w;
        } else {
            l = bbox[0] - bbox[2]/2.f - (INPUT_W - r_h * img.cols) / 2;
            r = bbox[0] + bbox[2]/2.f - (INPUT_W - r_h * img.cols) / 2;
            t = bbox[1] - bbox[3]/2.f;
            b = bbox[1] + bbox[3]/2.f;
            l = l / r_h;
            r = r / r_h;
            t = t / r_h;
            b = b / r_h;
        }
        return Rect(l, t, r-l, b-t);
    }

    float YoloTensorRT::iou(float lbox[4], float rbox[4])
    {
        float interBox[] = {
                max(lbox[0] - lbox[2]/2.f , rbox[0] - rbox[2]/2.f), //left
                min(lbox[0] + lbox[2]/2.f , rbox[0] + rbox[2]/2.f), //right
                max(lbox[1] - lbox[3]/2.f , rbox[1] - rbox[3]/2.f), //top
                min(lbox[1] + lbox[3]/2.f , rbox[1] + rbox[3]/2.f), //bottom
        };

        if(interBox[2] > interBox[3] || interBox[0] > interBox[1])
            return 0.0f;

        float interBoxS =(interBox[1]-interBox[0])*(interBox[3]-interBox[2]);
        return interBoxS/(lbox[2]*lbox[3] + rbox[2]*rbox[3] -interBoxS);
    }

    bool YoloTensorRT::cmp(const Yolo::Detection &a, const Yolo::Detection &b)
    {
        return a.det_confidence > b.det_confidence;
    }

    void YoloTensorRT::nms(vector<Yolo::Detection> &res, float *output, float nms_thresh)
    {
        map<float, vector<Yolo::Detection>> m;
        for (int i = 0; i < output[0] && i < Yolo::MAX_OUTPUT_BBOX_COUNT; i++)
        {
            if (output[1 + DETECTION_SIZE * i + 4] <= BBOX_CONF_THRESH) continue;
            Yolo::Detection det;
            memcpy(&det, &output[1 + DETECTION_SIZE * i], DETECTION_SIZE * sizeof(float));
            if (m.count(det.class_id) == 0) m.emplace(det.class_id, vector<Yolo::Detection>());
            m[det.class_id].push_back(det);
        }
        for (auto it = m.begin(); it != m.end(); it++)
        {
            //cout << it->second[0].class_id << " --- " << endl;
            auto& dets = it->second;
            sort(dets.begin(), dets.end(), cmp);
            for (size_t m = 0; m < dets.size(); ++m)
            {
                auto& item = dets[m];
                res.push_back(item);
                for (size_t n = m + 1; n < dets.size(); ++n)
                {
                    if (iou(item.bbox, dets[n].bbox) > nms_thresh)
                    {
                        dets.erase(dets.begin()+n);
                        --n;
                    }
                }
            }
        }
    }

    void YoloTensorRT::run()
    {
        cudaSetDevice(DEVICE);
        while (ros::ok())
        {
            ros::spinOnce();
        }

    }
}
