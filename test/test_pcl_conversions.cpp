#include <string>

#include "gtest/gtest.h"

#include "shm_msgs/pcl_conversions.h"

namespace {

class PCLConversionTests : public ::testing::Test {
protected:
  virtual void SetUp() {

    pcl_pc2.header.stamp = 3141592653;
    pcl_pc2.header.frame_id = "pcl";
    pcl_pc2.height = 1;
    pcl_pc2.width = 2;
    pcl_pc2.point_step = 1;
    pcl_pc2.row_step = 1;
    pcl_pc2.is_bigendian = true;
    pcl_pc2.is_dense = true;
    pcl_pc2.fields.resize(2);
    pcl_pc2.fields[0].name = "XYZ";
    pcl_pc2.fields[0].datatype = pcl::PCLPointField::INT8;
    pcl_pc2.fields[0].count = 3;
    pcl_pc2.fields[0].offset = 0;
    pcl_pc2.fields[1].name = "RGB";
    pcl_pc2.fields[1].datatype = pcl::PCLPointField::INT8;
    pcl_pc2.fields[1].count = 3;
    pcl_pc2.fields[1].offset = 8 * 3;
    pcl_pc2.data.resize(2);
    pcl_pc2.data[0] = 0x42;
    pcl_pc2.data[1] = 0x43;
  }

  pcl::PCLPointCloud2 pcl_pc2;
  shm_msgs::msg::PointCloud8k shm_pc2;
};

void test_pcl_pc(pcl::PCLPointCloud2 &pcl_pc) {
  EXPECT_EQ(std::string("pcl"), pcl_pc.header.frame_id);
  EXPECT_EQ(1U, pcl_pc.height);
  EXPECT_EQ(2U, pcl_pc.width);
  EXPECT_EQ(1U, pcl_pc.point_step);
  EXPECT_EQ(1U, pcl_pc.row_step);
  EXPECT_TRUE(pcl_pc.is_bigendian);
  EXPECT_TRUE(pcl_pc.is_dense);
  EXPECT_EQ("XYZ", pcl_pc.fields[0].name);
  EXPECT_EQ(pcl::PCLPointField::INT8, pcl_pc.fields[0].datatype);
  EXPECT_EQ(3U, pcl_pc.fields[0].count);
  EXPECT_EQ(0U, pcl_pc.fields[0].offset);
  EXPECT_EQ("RGB", pcl_pc.fields[1].name);
  EXPECT_EQ(pcl::PCLPointField::INT8, pcl_pc.fields[1].datatype);
  EXPECT_EQ(3U, pcl_pc.fields[1].count);
  EXPECT_EQ(8U * 3U, pcl_pc.fields[1].offset);
  EXPECT_EQ(2U, pcl_pc.data.size());
  EXPECT_EQ(0x42, pcl_pc.data[0]);
  EXPECT_EQ(0x43, pcl_pc.data[1]);
}

void test_shm_pc(shm_msgs::msg::PointCloud8k &shm_pc) {
  EXPECT_EQ(std::string("pcl"), shm_msgs::get_str(shm_pc.header.frame_id));
  EXPECT_EQ(1U, shm_pc.height);
  EXPECT_EQ(2U, shm_pc.width);
  EXPECT_EQ(1U, shm_pc.point_step);
  EXPECT_EQ(1U, shm_pc.row_step);
  EXPECT_TRUE(shm_pc.is_bigendian);
  EXPECT_TRUE(shm_pc.is_dense);
  EXPECT_EQ(2U, shm_pc.fields_size);
  EXPECT_EQ("XYZ", shm_msgs::get_str(shm_pc.fields[0].name));
  EXPECT_EQ(pcl::PCLPointField::INT8, shm_pc.fields[0].datatype);
  EXPECT_EQ(3U, shm_pc.fields[0].count);
  EXPECT_EQ(0U, shm_pc.fields[0].offset);
  EXPECT_EQ("RGB", shm_msgs::get_str(shm_pc.fields[1].name));
  EXPECT_EQ(pcl::PCLPointField::INT8, shm_pc.fields[1].datatype);
  EXPECT_EQ(3U, shm_pc.fields[1].count);
  EXPECT_EQ(8U * 3U, shm_pc.fields[1].offset);
  // EXPECT_EQ(2U, shm_pc.data.size());
  EXPECT_EQ(0x42, shm_pc.data[0]);
  EXPECT_EQ(0x43, shm_pc.data[1]);
}

TEST_F(PCLConversionTests, pointcloud2Conversion) {
  shm_msgs::fromPCL(pcl_pc2, shm_pc2);
  test_shm_pc(shm_pc2);
  pcl::PCLPointCloud2 pcl_pc2_2;
  shm_msgs::toPCL(shm_pc2, pcl_pc2_2);
  test_pcl_pc(pcl_pc2_2);
  EXPECT_EQ(pcl_pc2.header.stamp, pcl_pc2_2.header.stamp);
}

} // namespace


struct StampTestData
{
  const rclcpp::Time stamp_;
  rclcpp::Time stamp2_;

  explicit StampTestData(const rclcpp::Time &stamp)
    : stamp_(stamp)
  {
    std::uint64_t pcl_stamp;
    shm_msgs::toPCL(stamp_, pcl_stamp);
    shm_msgs::fromPCL(pcl_stamp, stamp2_);
  }
};

TEST(PCLConversionStamp, Stamps)
{
  {
    const StampTestData d(rclcpp::Time(1, 1000));
    EXPECT_TRUE(d.stamp_==d.stamp2_);
  }

  {
    const StampTestData d(rclcpp::Time(1, 999999000));
    EXPECT_TRUE(d.stamp_==d.stamp2_);
  }

  {
    const StampTestData d(rclcpp::Time(1, 999000000));
    EXPECT_TRUE(d.stamp_==d.stamp2_);
  }

  {
    const StampTestData d(rclcpp::Time(1423680574, 746000000));
    EXPECT_TRUE(d.stamp_==d.stamp2_);
  }

  {
    const StampTestData d(rclcpp::Time(1423680629, 901000000));
    EXPECT_TRUE(d.stamp_==d.stamp2_);
  }
}

int main(int argc, char **argv) {
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}
