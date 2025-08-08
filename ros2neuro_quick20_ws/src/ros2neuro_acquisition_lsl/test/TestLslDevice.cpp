#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <LSLDevice.hpp>

namespace rosneuro {
    class LSLDeviceTest : public LSLDevice {
        public:
            LSLDeviceTest() : LSLDevice() {}
            LSLDeviceTest(NeuroFrame* frame) : LSLDevice(frame) {}
            ~LSLDeviceTest() {}
            bool Setup(void) { return true; }
            bool Close(void) { return true; }
            bool Start(void) { return true; }
            bool Stop(void) { return true; }
            bool Open(void) { return true; }
    };

    class LSLDeviceTestSuite : public ::testing::Test {
        public:
            LSLDeviceTestSuite(){}
            ~LSLDeviceTestSuite(){}
            void SetUp(void) {
                lslDev = new LSLDeviceTest();
                lslDevFrame = new LSLDeviceTest(new NeuroFrame());
            }
            void TearDown(void) {
                delete lslDev;
                delete lslDevFrame;
            }
            LSLDevice* lslDev = new LSLDeviceTest();
            LSLDevice* lslDevFrame = new LSLDeviceTest(new NeuroFrame());
    };

    TEST_F(LSLDeviceTestSuite, Constructor) {
        EXPECT_EQ(lslDev->name_, "lsldev");
        EXPECT_EQ(lslDev->stream_, nullptr);
        EXPECT_EQ(lslDev->info_, nullptr);
        EXPECT_EQ(lslDev->stream_name_, "unknown");
        EXPECT_EQ(lslDev->stream_type_, "unknown");
        EXPECT_EQ(lslDevFrame->name_, "lsldev");
        EXPECT_EQ(lslDevFrame->stream_, nullptr);
        EXPECT_EQ(lslDevFrame->info_, nullptr);
        EXPECT_EQ(lslDevFrame->stream_name_, "unknown");
        EXPECT_EQ(lslDevFrame->stream_type_, "unknown");
    }

    TEST_F(LSLDeviceTestSuite, Configure) {
        NeuroFrame* frame = new NeuroFrame();
        unsigned int framerate = 1;

        ros::param::set("~stream_type", "unknown");
        ros::param::set("~stream_name", "unknown");

        EXPECT_EQ(lslDev->Configure(frame, framerate), true);
        EXPECT_EQ(lslDevFrame->Configure(frame, framerate), true);

        ros::param::del("~stream_name");
        EXPECT_FALSE(lslDev->Configure(frame, framerate));
        ros::param::del("~stream_type");
        EXPECT_FALSE(lslDev->Configure(frame, framerate));
    }

    TEST_F(LSLDeviceTestSuite, Setup) {
        ASSERT_TRUE(lslDev->Setup());
    }

    TEST_F(LSLDeviceTestSuite, Open) {
        ASSERT_TRUE(lslDev->Open());
    }

    TEST_F(LSLDeviceTestSuite, Close) {
        ASSERT_TRUE(lslDev->Close());
    }

    TEST_F(LSLDeviceTestSuite, Start) {
        ASSERT_TRUE(lslDev->Start());
    }

    TEST_F(LSLDeviceTestSuite, Stop) {
        ASSERT_TRUE(lslDev->Stop());
    }
}

int main(int argc, char **argv) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    ros::init(argc, argv, "test_lsl_device");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}