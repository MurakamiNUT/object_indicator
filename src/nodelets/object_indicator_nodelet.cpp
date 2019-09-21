#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <zbar.h>
#include <std_msgs/String.h>


namespace object_indicator{

    struct Tag
    {
        std::string message;
        std::vector<cv::Point> polygon;
    };

    using Tags = std::vector<Tag>;

    class disp_box : public nodelet::Nodelet{
        public:
            //起動処理
            virtual void onInit();
            //画像購読
            void image_callback(const sensor_msgs::ImageConstPtr& msg);
            //バウンディングボックス購読
            void boxes_callback(const darknet_ros_msgs::BoundingBoxes& msg);
            //処理画像配信
            void timer_callback(const ros::TimerEvent&);
        private:
            ros::NodeHandle nh_;        //ノード
            ros::NodeHandle pnh_;       //
            ros::Subscriber sub_image;  //画像購読者
            ros::Subscriber sub_boxes;  //バウンディングボックス購読者
            ros::Publisher pub_;        //処理画像配信者
            ros::Timer timer_;          //タイマ

            bool flg_img;
            bool flg_boxes;                    //配信フラグ
            
            //保存用
            cv::Mat mat;
            darknet_ros_msgs::BoundingBoxes boxes;
    };

    void disp_box::onInit(){
        nh_ = getNodeHandle();          //ノードハンドル取得
        pnh_ = getPrivateNodeHandle();
        flg_img = false;
        flg_boxes = false;
        //購読者立ち上げ
        sub_image = nh_.subscribe("raw_image", 10, &disp_box::image_callback, this);
        sub_boxes = nh_.subscribe("bounding_boxes", 10, &disp_box::boxes_callback, this);

        //配信者立ち上げ
        pub_ = nh_.advertise<sensor_msgs::Image>("disp_image", 10);

        //配信用タイマ立ち上げ
        timer_ = nh_.createTimer(ros::Duration(0.05), &disp_box::timer_callback, this);
    }


    void disp_box::image_callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;

        //ROS←→OpenCV間の変換
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& ex){
            NODELET_ERROR("Error");
            exit(-1);
        }
    
        mat = cv_ptr->image;

        if(flg_img!=true)flg_img=true;

        //sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst_mat).toImageMsg();
        //pub_.publish(pub_msg);
    }

    void disp_box::boxes_callback(const darknet_ros_msgs::BoundingBoxes& msg){
        //NODELET_INFO("recieve: %s", string_msg.data.c_str());
        boxes = msg;
        if(flg_boxes!=true)flg_boxes=true;
    }

    void disp_box::timer_callback(const ros::TimerEvent&){
        if(flg_img && flg_boxes){
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat dst_mat;
            int b[3] = {0, 0, 255};
            int g[3] = {0, 255, 0};
            int r[3] = {255, 0, 0};
            int color;

            int boxes_num = boxes.bounding_boxes.size();
            std::string obj_class;
            double obj_prob;
            int xmin, ymin, xmax, ymax;

            for(int num; num<boxes_num; num++){
                obj_class = boxes.bounding_boxes[num].Class;
                obj_prob = boxes.bounding_boxes[num].probability;
                xmin = boxes.bounding_boxes[num].xmin;
                ymin = boxes.bounding_boxes[num].ymin;
                xmax = boxes.bounding_boxes[num].xmax;
                ymax = boxes.bounding_boxes[num].ymax;
                obj_class += ":";
                obj_class += std::to_string(obj_prob);

                if(obj_prob > 0.9)color=2;
                else if(obj_prob > 0.7)color=1;
                else color=0;

                cv::rectangle(mat, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(255, 255, 255), 5, 4);
                cv::rectangle(mat, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(b[color], g[color], r[color]), 3, 4);
                putText(mat, obj_class, cv::Point(xmin, ymin), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3, 4);
                putText(mat, obj_class, cv::Point(xmin, ymin), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(b[color], g[color], r[color]), 1, 4);
            }
            sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg();
            pub_.publish(pub_msg);

            flg_img = false;
            flg_boxes = false;
        }

    }

    class motion_detection : public nodelet::Nodelet{
        public:
            //起動処理
            virtual void onInit();
            //画像購読・配信
            void image_callback(const sensor_msgs::ImageConstPtr& msg);
        private:
            ros::NodeHandle nh_;        //ノード
            ros::NodeHandle pnh_;       //
            ros::Subscriber sub_image;  //画像購読者
            ros::Publisher pub_;        //処理画像配信者
            
            //保存用
            cv::Mat past_mat;
            bool first_flg;
    };

    void motion_detection::onInit(){
        nh_ = getNodeHandle();          //ノードハンドル取得
        pnh_ = getPrivateNodeHandle();
        first_flg = false;
        //購読者立ち上げ
        sub_image = nh_.subscribe("raw_image", 10, &motion_detection::image_callback, this);

        //配信者立ち上げ
        pub_ = nh_.advertise<sensor_msgs::Image>("motion_image", 10);
        
    }


    void motion_detection::image_callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;

        //ROS←→OpenCV間の変換
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& ex){
            NODELET_ERROR("Error");
            exit(-1);
        }
    
        cv::Mat mat;
        cv::Mat mask_mat, motion_mat, pub_mat;
        cv::cvtColor(cv_ptr->image, mat, CV_BGR2GRAY);
        cv::Mat diff(mat.rows, mat.cols, mat.type());
        cv::Mat sub_img(mat.rows, mat.cols, CV_8UC3, cv::Scalar(255,0,255));

        if(first_flg!=true){
            first_flg=true;
            past_mat = mat.clone();
        }

        cv::absdiff(mat, past_mat, diff);
        cv::threshold(diff, diff, 80, 255, cv::THRESH_BINARY);

        mat.copyTo(past_mat);

        cv::cvtColor(diff, motion_mat, CV_GRAY2BGR);
        motion_mat = motion_mat-sub_img;
        diff = ~diff;
        cv::cvtColor(diff, mask_mat, CV_GRAY2BGR);
        cv_ptr->image.copyTo(pub_mat, mask_mat);

        sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_mat+motion_mat).toImageMsg();
        pub_.publish(pub_msg);
    }



    class qr_code_detection : public nodelet::Nodelet{
        public:
            //起動処理
            virtual void onInit();
            //画像購読・配信
            void image_callback(const sensor_msgs::ImageConstPtr& msg);
        private:
            ros::NodeHandle nh_;        //ノード
            ros::NodeHandle pnh_;       //
            ros::Subscriber sub_image;  //
            ros::Publisher pub_codes;   //QRコード名配信者
            ros::Publisher pub_image;   //処理画像配信者

            zbar::ImageScanner scanner;
            
    };

    void qr_code_detection::onInit(){
        nh_ = getNodeHandle();          //ノードハンドル取得
        pnh_ = getPrivateNodeHandle();
        //購読者立ち上げ
        sub_image = nh_.subscribe("raw_image", 3, &qr_code_detection::image_callback, this);

        //配信者立ち上げ
        pub_codes = nh_.advertise<std_msgs::String>("qr_codes", 10);
        pub_image = nh_.advertise<sensor_msgs::Image>("qr_codes_image", 3);
        
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    }


    void qr_code_detection::image_callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;

        //ROS←→OpenCV間の変換
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& ex){
            NODELET_ERROR("Error");
            exit(-1);
        }
        cv::Mat gray, color;
        color = cv_ptr->image.clone();
        cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
        int width = gray.cols;
        int height = gray.rows;
        zbar::Image img(width, height, "Y800", gray.data, width * height);
        scanner.scan(img);


        Tags tags;
        for(auto s = img.symbol_begin(); s != img.symbol_end(); ++s){
            Tag tag;
            tag.message = s->get_data();

            for(int i = 0; i < s->get_location_size(); i++)
                tag.polygon.push_back(
                        cv::Point(s->get_location_x(i), s->get_location_y(i)));
            tags.push_back(tag);
        }

        for (auto& tag : tags){
            //cv::polylines(color, tag.polygon, true, cv::Scalar(255, 255, 255), 5, 4);
            //cv::polylines(color, tag.polygon, true, cv::Scalar(255, 255, 0), 3, 4);
            int centre_x=0;
            int centre_y=0;
            for (auto poly : tag.polygon){
                centre_x += poly.x/tag.polygon.size();
                centre_y += poly.y/tag.polygon.size();
            }
            cv::circle(color, cv::Point(centre_x, centre_y), 20, cv::Scalar(255, 255, 255), 5, 4);
            cv::circle(color, cv::Point(centre_x, centre_y), 20, cv::Scalar(255, 255, 0), 3, 4);
            putText(color, tag.message, tag.polygon[0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3, 4);
            putText(color, tag.message, tag.polygon[0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 0), 1, 4);
            pub_codes.publish(tag.message);
        }
        sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
        pub_image.publish(pub_msg);
    }

} // namespace plugin_lecture
PLUGINLIB_EXPORT_CLASS(object_indicator::disp_box, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(object_indicator::motion_detection, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(object_indicator::qr_code_detection, nodelet::Nodelet);
