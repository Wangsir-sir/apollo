APOLLO_MAIN(){
    // 1. init and start the can card hardwarejiekou
    //-- can_client: ESD/Kvaser
    can_client_->Start();
    // 2. start receive first then send
    //-- can_receiver_.Start();
    std::unique_ptr<std::thread> thread_;
    MessageManager<SensorType> *pt_manager_; 
    //-- 解析conti_radar_conf.pb.txt
    GetProtoFromFile(FLAGS_sensor_conf_file,&conti_radar_conf_);
    can_type_ = conti_radar_conf_.can_conf().can_card_parameter();
    can_client_ = can_factory->CreateCANClient(can_type_); //-- 这里以kvaser为例
    //-- 这是很重要的报文解析类的管理类
    ContiRadarMessageManager *pt_manager_ = new ContiRadarMessageManager();
    
    Run in std::thread{
        wile(IsRunning()){
            can_client_->Recieve(&buf);
            for (const auto &frame : buf) {
                uint8_t len = frame.len;
      			uint32_t uid = frame.id;
                //-- 报文解析类进行报文解析以及ROS的publish等操作
                pt_manager_->Parse(uid, data, len);
            }
        }
    }
}