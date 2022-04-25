//============================================================================
// Name        : compass.cpp
// Author      : Kirols Bakheat
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cstdint>

#define BUFSIZE 512

#define SERIAL_PATH "/dev/SPARTON_uart"

// #define START_STR "1 accelp.p " \
//                   "1 gyrop.p "  \
//                   "1 quat.p "   \
//                   "\r"

// #define END_STR   \
//     "0 accelp.p " \
//     "0 gyrop.p "  \
//     "0 quat.p "   \
//     "\r"
#define REQUEST_QUATERNION "$PSPA,QUAT\r\n"
#define REQUEST_GYRO "$PSPA,G\r\n"
const uint8_t REQUEST_ACCELERATION[] = {0xA4, 0x07, 0xA0};
/**
 * @brief 
 * Make sure this matrix is exactly 9 doubles long (72 bytes).
 * There are 2 unsafe memory copies involving this matrix.
 * I copied this from the python driver but couldn't verify it from the documentation.
 */
const double COVARIANCE_MATRIX[] = {
    1e-9, 0, 0,
    0, 1e-9, 0,
    0, 0, 1e-9};

const double
    // The conversion from the raw acceleration data to m/s^2 (which ros uses)
    // To get this, send the sparton command "accelrange di.\r" and it will respond with either 4g or 8g. 4g maps to 2048 and 8g maps to 1024.
    // The 9.8 is gs to m/s^2
    //(1 / (1000.0 * 9.80665))
    ACCEL_CONVERSION = 0.00010197,
    // This is the sampling rate of the Gyroscope, can be retrieved by sending the sparton command "gyroSampleRate di.\r" and reading its response
    RADS_PER_SAMPLE_TO_RADS_PER_SECOND = 105.250000,
    MILLIDEGREES_PER_SECOND_TO_RADS_PER_SECOND = 0.01;

typedef struct termios Termios;

uint8_t buf[BUFSIZE] = {0};
sensor_msgs::Imu imu_msg;

bool
    error = false,
    new_quat = false,
    new_accel = false,
    new_gyro = false;

int err = 0;

int get_quaternion(int tty_fd);
int get_accel(int tty_fd);
int get_gyro(int tty_fd);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "compass");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/localization/imu/raw", 1);
    ros::Rate loop_rate(20);

    memcpy(imu_msg.linear_acceleration_covariance.c_array(), COVARIANCE_MATRIX, sizeof(COVARIANCE_MATRIX));
    memcpy(imu_msg.angular_velocity_covariance.c_array(), COVARIANCE_MATRIX, sizeof(COVARIANCE_MATRIX));
    memcpy(imu_msg.orientation_covariance.c_array(), COVARIANCE_MATRIX, sizeof(COVARIANCE_MATRIX));

    Termios
        compass,
        old_compass;

    int tty_fd;

    uint8_t c;
    bool error = false, set = false;

    /************ setting up compass Interface ************/

    tty_fd = open(SERIAL_PATH, O_RDWR);
    tcgetattr(tty_fd, &old_compass);
    memcpy(&compass, &old_compass, sizeof(compass));
    // memset(&compass, 0, sizeof(compass));
    cfmakeraw(&compass);
    cfsetispeed(&compass, B115200);
    cfsetospeed(&compass, B115200);
    compass.c_cflag |= (CREAD | CS8);
    compass.c_cflag &= ~(PARENB | CSTOPB);
    compass.c_iflag |= (ICRNL); // ICRNL so we can send \r without it being changed and ICANON so we can read one line at a time
    compass.c_cc[VMIN] = 1;
    compass.c_cc[VTIME] = 0;
    // compass.c_cc[VMIN] = 0;
    // compass.c_cc[VTIME] = 0;

    tcsetattr(tty_fd, TCSAFLUSH, &compass);

    /************ Read/Write Logic ************/

    while (ros::ok())
    {
        // ROS_INFO("Starting");
        if (get_quaternion(tty_fd))
        {
            error = true;
            break;
        }
        // ROS_INFO("Finished Quaternion");
        if (get_accel(tty_fd))
        {
            error = true;
            break;
        }
        // ROS_INFO("Finished Acceleration");
        if (get_gyro(tty_fd))
        {
            error = true;
            break;
        }
        // ROS_INFO("Finished Gyro");
	imu_msg.header.stamp = ros::Time::now();
        pub.publish(imu_msg);
        // ROS_INFO("Published");
        loop_rate.sleep();
    }

    ROS_INFO("Closing serial port");
    tcflush(tty_fd, TCIFLUSH);
    tcdrain(tty_fd);
    tcsetattr(tty_fd, TCSANOW, &old_compass);
    close(tty_fd);
    // tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdo);

    /************ Program termination************/
    if (error)
    {
        ROS_ERROR("Error occured in compass driver");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("Compass driver terminated successfully");
        return EXIT_SUCCESS;
    }
}

int get_quaternion(int tty_fd)
{
    uint8_t c;
    int index = 0;
    tcflush(tty_fd,TCIOFLUSH);
    write(tty_fd, REQUEST_QUATERNION, sizeof(REQUEST_QUATERNION) - 1);
    while (true)
    {
        err = read(tty_fd, &c, 1);
        if (c == '\n')
        {
            sscanf((const char *const)buf, "$PSPA,QUATw=%lf,x=%lf,y=%lf,z=%lf*%*s\n", &imu_msg.orientation.w, &imu_msg.orientation.x, &imu_msg.orientation.y, &imu_msg.orientation.z);
            memset(buf, 0, index);
            return 0;
        }
        else
        {
            buf[index] = c;
            index++;
            if (index >= BUFSIZE)
            {
                ROS_ERROR("Buffer overflow in get_quaternion");
                return 1;
            }
        }
        if (err < 0)
        {
            ROS_ERROR("Error reading from serial port: %s", strerror(errno));
            return 1;
        }
    }
}

int get_gyro(int tty_fd)
{
    uint8_t c;
    int index = 0;
    tcflush(tty_fd,TCIOFLUSH);
    write(tty_fd, REQUEST_GYRO, sizeof(REQUEST_GYRO) - 1);
    while (true)
    {
        err = read(tty_fd, &c, 1);
        if (c == '\n')
        {
            sscanf((const char *const)buf, "$PSPA,Gx=%lf,Gy=%lf,Gz=%lf%*s", &imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y, &imu_msg.angular_velocity.z);
            imu_msg.angular_velocity.x *= MILLIDEGREES_PER_SECOND_TO_RADS_PER_SECOND;
            imu_msg.angular_velocity.y *= MILLIDEGREES_PER_SECOND_TO_RADS_PER_SECOND;
            imu_msg.angular_velocity.z *= MILLIDEGREES_PER_SECOND_TO_RADS_PER_SECOND;

            memset(buf, 0, index);
            return 0;
        }
        else if (err == 0)
        {
            ROS_INFO("No data received from gyro");
            continue;
        }
        else
        {
            buf[index] = c;
            index++;
            if (index >= BUFSIZE)
            {
                ROS_ERROR("Buffer overflow in get_gyro");
                return 1;
            }
        }
        if (err < 0)
        {
            ROS_ERROR("Error reading from serial port: %s", strerror(errno));
            return 1;
        }
    }
}

//! Byte swap short
int16_t swap_int16( int16_t val ) 
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

int get_accel(int tty_fd)
{
    int16_t x, y, z, total;
    // uint8_t checksum;
    static const int RESPONSE_LENGTH = 11;
    // uint8_t *buf_ptr = buf;
    tcflush(tty_fd,TCIOFLUSH);
    if (write(tty_fd, REQUEST_ACCELERATION, sizeof(REQUEST_ACCELERATION)) != sizeof(REQUEST_ACCELERATION))
    {
        ROS_ERROR("Error writing to serial port: %s", strerror(errno));
        return 1;
    }
    for (uint8_t *buf_ptr = buf; buf_ptr < (buf + RESPONSE_LENGTH - 1); buf_ptr += err)
    {
        err = read(tty_fd, buf_ptr, 1);
        if (err < 0)
        {
            ROS_ERROR("Error reading from serial port: %s", strerror(errno));
            return 1;
        }
        else if (*buf_ptr == '\n'){
            ROS_ERROR("Recieved an early \\n");
            return get_accel(tty_fd);
        }
    }

    x = swap_int16(*(uint16_t *)(buf + 2));
    y = swap_int16(*(uint16_t *)(buf + 4));
    z = swap_int16(*(uint16_t *)(buf + 6));
    // total = *(int16_t *)(buf + 8);
    // checksum = *(uint8_t *)(buf + 10);
    imu_msg.linear_acceleration.x = x * ACCEL_CONVERSION;
    imu_msg.linear_acceleration.y = y * ACCEL_CONVERSION;
    imu_msg.linear_acceleration.z = z * ACCEL_CONVERSION;

    memset(buf, 0, RESPONSE_LENGTH);
    return 0;
}
