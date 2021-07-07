#include "gtest/gtest.h"

#include "modules/bridge/common/protobuf_manager.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace bridge {

TEST(ProtobufManagerTest, CreateObj){
    ProtobufManager<::apollo::drivers::PointCloud> protoManager;
    auto ptr1 = protoManager.CreateObj(1);
    auto ptr2 = protoManager.CreateObj(1);
    auto ptr3 = protoManager.CreateObj(2);
    EXPECT_NE(ptr1, nullptr);
    EXPECT_EQ(ptr1, ptr2);
    EXPECT_NE(ptr1, ptr3);
}

TEST(ProtobufManagerTest, DestructObj){
    ProtobufManager<::apollo::drivers::PointCloud> protoManager;
    auto ptr1 = protoManager.CreateObj(1);
    EXPECT_TRUE(protoManager.DestructObj(1));
    EXPECT_FALSE(protoManager.DestructObj(2));
}

TEST(ProtobufManagerTest, getTimeStamp){
    ProtobufManager<::apollo::drivers::PointCloud> protoManager;
    auto ptr1 = protoManager.CreateObj(1);
    auto ptr2 = protoManager.CreateObj(1);
    auto ptr3 = protoManager.CreateObj(2);
    EXPECT_NE(protoManager.getTimeStamp(1), protoManager.getTimeStamp(2));
}


}  // namespace bridge
}  // namespace apollo
