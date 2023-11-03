#include <iostream>
#include <thread>
#include <queue>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/support/async_stream.h>

// Sensor Error(Gausian) Modeling
#include <random>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensorview_rpc.grpc.pb.h"
#include "task/task.h"

using grpc::Channel;
using grpc::ClientAsyncReader;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;

using CameraReturn = std::tuple<int, size_t, sensor_msgs::CompressedImage>;

// config 읽어와서 publisher / converting thread seq table, publish thread seq table 설정
// ros::Publisher pub_img_1, pub_img_2, pub_cloud;
// std::vector<ros::Publisher> pub_imgs_, pub_clouds_;
// int num_of_camera_, num_of_lidar_;

class SensorViewClient
{
public:
  SensorViewClient(std::shared_ptr<Channel> channel)
      : stub_(SensorViewRPC::NewStub(channel)), state_(CallState::NewCall)
  {
    
    XmlRpc::XmlRpcValue camera_param, lidar_param;
    nh_.getParam("camera_param", camera_param);
    nh_.getParam("lidar_param", lidar_param);

    std::cout << "get param" << std::endl;

    num_of_camera_ = camera_param.size();
    num_of_lidar_ = lidar_param.size();

    pub_imgs_.resize(num_of_camera_);
    pub_clouds_.resize(num_of_lidar_);

    for (size_t i = 0; i < camera_param.size(); i++){
      std::string topic_name = "camera" + std::to_string(i) + "/grpc/compressed";
      pub_imgs_[i] = nh_.advertise<sensor_msgs::CompressedImage>(topic_name, 1);
    }

    for (size_t i = 0; i < lidar_param.size(); i++){
      std::string topic_name = "lidar" + std::to_string(i) + "/grpc/points";
      pub_clouds_[i] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    }

    std::cout << "finished" << std::endl;

    this->onNext(true);
  }

  ~SensorViewClient()
  {
    void *ignoredTag = nullptr;
    bool ok = false;
    while (cq_.Next(&ignoredTag, &ok));

    call_->reader->Finish(&call_->status, call_);
  }

  enum class CallState {
    NewCall,
    SendingRequest,
    ReceivingData,
    CallComplete
  };

  struct MsgResult
  {
    // std::unordered_map<int, std::map<size_t,sensor_msgs::PointCloud2>> lidar_res_table;
    std::unordered_map<int, std::queue<std::future<std::pair<size_t,sensor_msgs::CompressedImage>>>> camera_res_table; // seq: 디버깅용 -> 빼도 상광없음
  };

  void handleNewCallState() {
    call_ = new AsyncClientCall;
    call_->request.set_host_name("localhost:50051");

    call_->reader = stub_->PrepareAsyncGetSensorView(&call_->context, call_->request, &cq_);

    // StartCall initiates the RPC call
    state_ = CallState::SendingRequest;
    call_->reader->StartCall(call_);
  }

  void handleSendingRequestState() {
    state_ = CallState::ReceivingData;

    call_->reader->Read(&call_->reply, call_);    
  }

  void handleReceivingDataState() {
    static size_t num_data = 0;
    const auto &sensor_view = call_->reply;

    std::chrono::time_point<std::chrono::system_clock> now =
          std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto current_nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count()*1e-9;

    double sensor_view_stamp = sensor_view.timestamp().seconds()+sensor_view.timestamp().nanos()*1e-9;
    double communication_time = current_nanoseconds - sensor_view_stamp;
    // std::cout.precision(3);
    // std::cout << std::fixed << "[sensor, current] : " << sensor_view_stamp << ", " << current_nanoseconds << std::endl;
    // std::cout << "[msg size, communication speed] : " << sensor_view.ByteSizeLong() << ", " << (8.*sensor_view.ByteSizeLong()/(1024*1024))/communication_time << "Mbps" << std::endl;
    
    {
      std::lock_guard<std::mutex> lock(sensor_view_mutex_);
      sensor_view_buf_.push(sensor_view);
    }

    call_->reply.Clear();
    call_->reader->Read(&call_->reply, call_);

  }

  void handleCallCompleteState()
  {
    switch (call_->status.error_code())
    {
    case grpc::OK:
      std::cout << "[" << call_->request.host_name() << "]: RPC completed" << std::endl;
      break;

    case grpc::CANCELLED:
      std::cout << "[" << call_->request.host_name() << "]: RPC cancelled" << std::endl;
      break;

    default:
      std::cout << "[" << call_->request.host_name() << "]: RPC failed: " << call_->status.error_message() << std::endl;
      break;
    }
  }

  bool onNext(bool ok)
  {
    try
    {
      if (ok)
      {
        if (state_ == CallState::NewCall)
        {
          std::cout << "NewCall" << std::endl;
          this->handleNewCallState();
        }
        else if (state_ == CallState::SendingRequest)
        {
          std::cout << "SendingRequest" << std::endl;
          this->handleSendingRequestState();
        }
        else if (state_ == CallState::ReceivingData)
        {
          this->handleReceivingDataState();
        }
        else if (state_ == CallState::CallComplete)
        {
          std::cout << "CallComplete" << std::endl;
          this->handleCallCompleteState();
          return false;
        }
      }
      else
      {
        state_ = CallState::CallComplete;
        call_->reader->Finish(&call_->status, call_);
      }
    }
    catch (std::exception &e)
    {
      gpr_log(GPR_ERROR, "Processing error: %s", e.what());
    }
    catch (...)
    {
      gpr_log(GPR_ERROR, "Processing error: unknown exception caught");
    }

    return true;
  }

  void AsyncCompleteRpc()
  {
    void *got_tag;
    bool ok = false;

    // Block until the next result is available in the completion queue "cq".
    while (1)
    {
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      cq_.Next(&got_tag, &ok);

      auto duration = now.time_since_epoch();
      auto micros =
          std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
      // std::cout << "[" << micros << "], Next!!" << std::endl;
      AsyncClientCall *call = static_cast<AsyncClientCall *>(got_tag);      
      if(!ok) continue;
      if (!this->onNext(ok))
      {
        break;
      }
    }
  }

  static void ProcLidar(std::shared_ptr<LidarSensorView> lidar_sensor_view,
                        int lidar_id, size_t seq){
                  // std::shared_ptr<TestMsg> msg){
    // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    // std::cout << "converting start" << std::endl;

    if(seq % 2 == 1){
      ros::Duration(1.0).sleep();
    }

    // auto lidar_sensor_view = msg->lidar_sensor_view_;
    // auto lidar_id = msg->lidar_id_;
    // auto seq = msg->seq_;

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "velodyne";
    // cloud_msg.header.frame_id = "hero/lidar";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.width = lidar_sensor_view->view_configuration().directions_size();
    cloud_msg.height = 1;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step*cloud_msg.width*cloud_msg.height;

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    int speed_of_light = 299792458; // m/s
    for ( size_t i = 0; i < lidar_sensor_view->view_configuration().directions_size(); i++ ){
      auto direction = lidar_sensor_view->view_configuration().directions()[i];
      auto reflection = lidar_sensor_view->reflection()[i];

      float magnitude = reflection.time_of_flight()*speed_of_light;
      // std::normal_distribution<> distr(0.0, 0.1);
      // std::random_device rd;
      // std::mt19937 gen(rd());
      // float error = distr(gen); 
      // magnitude += error;

      unsigned char bytes[4];
      float x = magnitude*direction.x();
      memcpy(bytes, &x, sizeof x);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);
      float y = magnitude*direction.y();
      memcpy(bytes, &y, sizeof y);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);        
      float z = magnitude*direction.z();
      memcpy(bytes, &z, sizeof z);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);        
      float intensity = reflection.signal_strength();
      memcpy(bytes, &intensity, sizeof intensity);
      cloud_msg.data.insert(cloud_msg.data.end(), bytes, bytes+4);
    }

    // std::chrono::duration<double> running_time = std::chrono::system_clock::now() - now;
    // std::cout.precision(3);
    // std::cout << std::fixed << "lidar converting running time : " << running_time.count() * 1000 << "ms" << std::endl;

    // ros::Duration(2.0).sleep();
    // std::cout << "converting" << std::endl; 
    
    // msg_result_.lidar_res_table[lidar_id][seq] = cloud_msg;

    std::cout << "converting finished lidar id : " << lidar_id << ", seq : " << seq << std::endl;
  }

  static std::pair<size_t,sensor_msgs::CompressedImage> ProcCamera(std::shared_ptr<CameraSensorView> camera_sensor_view,
                                                 int camera_id, size_t seq){
    std::cout << "ProcCamera start camera : " << camera_id << ", seq : " << seq << std::endl;
    // std::cout << "ProcCamera start camera : " << camera_id << std::endl;

    sensor_msgs::CompressedImage img;
    img.header.stamp = ros::Time::now();
    img.format = "png";
    img.data = {camera_sensor_view->image_data().begin(),
                camera_sensor_view->image_data().end()};


    {
      // std::lock_guard<std::mutex> lock(res_mutex_);
      // std::cout << "camera " << camera_id << " seq : " << seq << " add start" << std::endl;
      // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      // input_flag_table[seq] = false;
      // msg_result_.camera_res_table[camera_id][seq].first = false;

      // msg_result_.camera_res_table[camera_id][seq].second = img;
      // input_flag_table[seq] = true;
      // msg_result_.camera_res_table[camera_id][seq].first = true;

      // std::chrono::duration<double> inserting_time = std::chrono::system_clock::now() - now;
      // std::cout.precision(3);
      // std::cout << std::fixed << "inserting time : " << inserting_time.count() * 1000 << "ms" << std::endl;
      // std::cout << "camera " << camera_id << " seq : " << seq << " add end" << std::endl;
    }

    std::cout << "ProcCamera end camera : " << camera_id << ", seq : " << seq << std::endl;
    // std::cout << "ProcCamera end camera : " << camera_id << std::endl;

    return std::make_pair(seq,img);
  }

  void ConvertThread(){ 
    std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table;
    
    for(int i = 0 ; i < num_of_lidar_ ; i++){
      lidar_seq_table[i] = 0;  
    }

    for(int i = 0 ; i < num_of_camera_ ; i++){
      camera_seq_table[i] = 0;  
    }

    while(1){
      SensorView sensor_view;
      if(sensor_view_buf_.size() != 0){
          std::lock_guard<std::mutex> lock(sensor_view_mutex_);
          sensor_view = sensor_view_buf_.front();
          sensor_view_buf_.pop();
      } else continue;

      // std::vector<std::future<sensor_msgs::CompressedImage>> camera_results_future;
      // std::vector<std::future<sensor_msgs::PointCloud2>> lidar_results_future;

      // std::vector<sensor_msgs::PointCloud2> lidar_results;

      // std::cout << "lidar_sensor_view size : " << sensor_view.lidar_sensor_view().size() << std::endl;
      // for ( int i = 0 ; i < sensor_view.lidar_sensor_view().size() ; i++ ) {
      //   auto lidar_sensor_view = std::make_shared<LidarSensorView>(sensor_view.lidar_sensor_view(i));
      //   int lidar_id = lidar_sensor_view->view_configuration().sensor_id().value();

      //   keti::task::Async(&SensorViewClient::ProcLidar, lidar_sensor_view, lidar_id, lidar_seq_table[lidar_id]);
      //   std::cout << "lidar id " << lidar_id <<  " seq : " << lidar_seq_table[lidar_id] << std::endl;

      //   lidar_seq_table[lidar_id]++;
      // }

      // std::cout << "camera_sensor_view size : " << sensor_view.camera_sensor_view().size() << std::endl;
      for ( int i = 0 ; i < sensor_view.camera_sensor_view().size() ; i++ ) {
        auto camera_sensor_view = std::make_shared<CameraSensorView>(sensor_view.camera_sensor_view(i));
        int camera_id = camera_sensor_view->view_configuration().sensor_id().value();
        size_t seq = camera_seq_table[camera_id];

        // std::cout << "start Add Conv Thread camera id " << camera_id <<  " seq : " << camera_seq_table[camera_id] << std::endl;
        // camera_future_list_.push_back(keti::task::Async(&SensorViewClient::ProcCamera, camera_sensor_view, camera_id, camera_seq_table[camera_id]));
        // msg_result_.camera_res_table[camera_id][seq] = keti::task::Async(&SensorViewClient::ProcCamera, camera_sensor_view, camera_id, seq);
        msg_result_.camera_res_table[camera_id].push(keti::task::Async(&SensorViewClient::ProcCamera, camera_sensor_view, camera_id, seq));
        // std::cout << "finish Add Conv Thread camera id " << camera_id <<  " seq : " << camera_seq_table[camera_id] << std::endl;

        camera_seq_table[camera_id]++;
      }


      // std::chrono::duration<double> running_time = std::chrono::system_clock::now() - now;
      // std::cout.precision(3);
      // std::cout << std::fixed << "converting running time : " << running_time.count() * 1000 << "ms" << std::endl;
    }
  }

  void PublishThread(){
    // std::unordered_map<int,size_t> lidar_seq_table, camera_seq_table;
    // for(int i = 0 ; i < num_of_lidar_ ; i++){
    //   lidar_seq_table[i] = 0;  
    // }

    // for(int i = 0 ; i < num_of_camera_ ; i++){
    //   camera_seq_table[i] = 0;  
    // }
    int test = 0;

    while(true){
      if(test < 100){
        std::cout << "start publish thread" << std::endl;
        test++;
      }
      
      // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      // if(camera_future_list_.size() > 0){
      //   auto camera_res = camera_future_list_.front().get();
      //   camera_future_list_.pop_front();
      //   msg_result_.camera_res_table[std::get<0>(camera_res)][std::get<1>(camera_res)] = std::get<2>(camera_res);
      // }

      // for(auto it = camera_future_list_.begin() ; it != camera_future_list_.end() ; it++){
      //   std::cout << "camera_future_list_ size : " << camera_future_list_.size() << std::endl;
      //   auto camera_res = it->get();
      //   it = camera_future_list_.erase(it);
      //   msg_result_.camera_res_table[std::get<0>(camera_res)][std::get<1>(camera_res)] = std::get<2>(camera_res);
      // }

      // for(int i = 0 ; i < num_of_lidar_ ; i++){
      //   if(msg_result_.lidar_res_table.count(i) == 0){
      //     continue;
      //   }

      //   if(msg_result_.lidar_res_table[i].count(lidar_seq_table[i]) == 0){
      //     continue;
      //   }

      //   std::cout << "try to publish lidar topic" << std::endl;
        
      //   pub_clouds_[i].publish(msg_result_.lidar_res_table[i][lidar_seq_table[i]]);

      //   std::cout << "publish finished lidar id : " << i <<  " seq : " << lidar_seq_table[i] << std::endl;

      //   msg_result_.lidar_res_table[i].erase(lidar_seq_table[i]);
      //   lidar_seq_table[i]++;
      //   std::cout << "result size : " << msg_result_.lidar_res_table[i].size() << std::endl;
      // }

      for(int i = 0 ; i < num_of_camera_ ; i++){
        // if(input_flag_table.count(camera_seq_table[i]) == 0){
        //   continue;
        // }

        // if(!input_flag_table[camera_seq_table[i]]){
        //   continue;
        // }

        if(msg_result_.camera_res_table.count(i) == 0){
          continue;
        }

        if(msg_result_.camera_res_table[i].size() == 0){
          continue;
        }

        // if(!msg_result_.camera_res_table[i][camera_seq_table[i]].first){
        //   continue;
        // }

        // sensor_msgs::CompressedImage msg;

        // {
          // std::lock_guard<std::mutex> lock(res_mutex_);
          // std::cout << "before getting camera msg : " << i << "seq : " << camera_seq_table[i] << std::endl;
          // msg = msg_result_.camera_res_table[i][camera_seq_table[i]];
          // std::cout << "after getting camera msg : " << i << "seq : " << camera_seq_table[i] << std::endl;
        // }

        // std::cout << "try to publish camera topic : " << i << "seq : " << camera_seq_table[i] << std::endl;
        // std::chrono::duration<double> before_pub_time = std::chrono::system_clock::now() - now;
        // std::cout.precision(3);
        // std::cout << std::fixed << "before publish time : " << before_pub_time.count() * 1000 << "ms" << std::endl;

        // std::cout << "try to publish camera topic : " << i << "seq : " << camera_seq_table[i] << std::endl;
        std::cout << "try to publish camera topic : " << i << std::endl;
        // std::chrono::duration<double> before_pub_time = std::chrono::system_clock::now() - now;
        // std::cout.precision(3);
        // std::cout << std::fixed << "before publish time : " << before_pub_time.count() * 1000 << "ms" << std::endl;
        // auto msg = msg_result_.camera_res_table[i][camera_seq_table[i]].get();
        auto msg = msg_result_.camera_res_table[i].front().get();

        // pub_imgs_[i].publish(msg_result_.camera_res_table[i][camera_seq_table[i]].second);

        // pub_imgs_[i].publish(msg_result_.camera_res_table[i][camera_seq_table[i]]);
        std::cout << "check camera id : " << i << " seq : " << msg.first << std::endl; 
        
        pub_imgs_[i].publish(msg.second);

        // std::chrono::duration<double> after_pub_time = std::chrono::system_clock::now() - now;
        // std::cout << std::fixed << "after publish time : " << after_pub_time.count() * 1000 << "ms" << std::endl;

        // std::cout << "publish finished camera id : " << i <<  " seq : " << camera_seq_table[i] << std::endl;
        std::cout << "publish finished camera id : " << i << std::endl;

        {
          std::cout << "camera " << i << " erase start" << std::endl;
          // msg_result_.camera_res_table[i].erase(camera_seq_table[i]);
          msg_result_.camera_res_table[i].pop();
          // auto num_of_erase = input_flag_table.erase(camera_seq_table[i]);
          // std::cout << "camera flag " << i << " erased num : " << num_of_erase << std::endl;
          std::cout << "camera " << i << " erase end" << std::endl;
        }

        // camera_seq_table[i]++;
        std::cout << "result size : " << msg_result_.camera_res_table[i].size() << std::endl;
        // std::cout << "flag table size : " << input_flag_table.size() << std::endl;
      }

    }
  }

private:
  struct AsyncClientCall
  {
    Request request;

    // Container for the data we expect from the server.
    SensorView reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // Storage for the status of the RPC upon completion.
    Status status;

    // std::unique_ptr<ClientAsyncReader<SensorView>> reader;
    std::unique_ptr<ClientAsyncReader<SensorView>> reader;
  };

private:
  AsyncClientCall *call_;
  std::unique_ptr<SensorViewRPC::Stub> stub_;
  CompletionQueue cq_;
  CallState state_;
  std::queue<SensorView> sensor_view_buf_;
  std::mutex sensor_view_mutex_;
  std::list<std::future<CameraReturn>> camera_future_list_;
  MsgResult msg_result_;
  
  // static MsgResult msg_result_;
  static std::mutex res_mutex_;

  // ros
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> pub_imgs_, pub_clouds_;
  ros::Publisher pub_img_1_, pub_img_2_;
  int num_of_camera_, num_of_lidar_;

};

// SensorViewClient::MsgResult SensorViewClient::msg_result_;
std::mutex SensorViewClient::res_mutex_;

int main(int argc, char** argv){
  ros::init(argc, argv, "grpc_client");

  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(1 * 1024 * 1024 * 1024);
  SensorViewClient client(
      grpc::CreateCustomChannel("localhost:50051",
                                grpc::InsecureChannelCredentials(),
                                args));

  ros::Duration(0.1).sleep();

  std::thread thread_ = std::thread(&SensorViewClient::AsyncCompleteRpc, &client);
  std::thread converting_thread_ = std::thread(&SensorViewClient::ConvertThread, &client);
  std::thread publish_thread_ = std::thread(&SensorViewClient::PublishThread, &client);

  thread_.join();
  converting_thread_.join();
  publish_thread_.join();

  return 0;
}