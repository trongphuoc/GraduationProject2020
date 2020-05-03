
#include <errno.h>

/* Verification Test Environment Include Files */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>
#include <malloc.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

extern "C"
{
#include <jpeglib.h>
}
#include <jconfig.h>
#include <jerror.h>
#include <jmorecfg.h>
#include <turbojpeg.h>
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include <iostream>
#include "hps_0.h"

#define BUFFER_TEST_NUM 3
#define SAT(c)       \
    if (c & (~255))  \
    {                \
        if (c < 0)   \
            c = 0;   \
        else         \
            c = 255; \
    }
#define SDRAM_BASE_ADDR 0
#define ALT_VIP_SOFTWARE_RESET_N_BASE 0x00000200 //

// #define ALT_STM_OFST (0xfc000000)
// #define ALT_LWFPGASLVS_OFST (0xff200000)  // axi_lw_master

// #define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
// #define HW_FPGA_AXI_SPAN (0x40000000)      // Bridge span
// #define HW_FPGA_AXI_MASK (HW_FPGA_AXI_SPAN - 1)

// #define ALT_GPIO1_BASE_OFST (0xFF709000)

// #define HW_REGS_BASE (ALT_STM_OFST)
// #define HW_REGS_SPAN (0x04000000)
// #define HW_REGS_MASK (HW_REGS_SPAN - 1)

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define VIDEO_BYTE_PER_PIXEL 4 //changed to sit one color
#define VIDEO_FRAME_PIXEL_NUM (VIDEO_WIDTH * VIDEO_HEIGHT)
#define VIDEO_FRAME_MEM_SIZE (VIDEO_FRAME_PIXEL_NUM * VIDEO_BYTE_PER_PIXEL)

// #define DEMO_VGA_FRAME0_ADDR 0x00000000 //0x00080000 //0x00100000  //on chip memory base
// #define FR0_FRAME0_OFFSET (0x00000000)
// #define FR0_FRAME1_OFFSET (0x00800000) //???

using namespace std;
using namespace cv;

struct image_buffer
{
    unsigned char *start;
    unsigned int length;
    size_t offset;
};

struct image_buffer buffers[BUFFER_TEST_NUM];
int video_out_width = 640;
int video_out_heigh = 480;
int video_out_fmt = V4L2_PIX_FMT_YUYV;
char v4l2_device_file[50] = "/dev/video0";
char jpeg_output[50] = "Capture.jpg";

// //base addr
// static volatile unsigned long *gpio1_addr = NULL;
// static volatile unsigned long *h2p_lw_axi_addr = NULL;
// static volatile unsigned long *h2p_lw_led_addr = NULL;
// static volatile unsigned long *h2p_vip_frame_reader0_addr = NULL;
// //static volatile unsigned long *h2p_vip_frame_reader1_addr = NULL;
// static volatile unsigned long *h2p_memory_addr = NULL;
// static volatile unsigned long *h2p_onchip_memory_addr = NULL;
// static volatile unsigned long *h2p_vip_mix_addr = NULL;

// void detectAndDraw( Mat& img, CascadeClassifier& cascade,
//                     CascadeClassifier& nestedCascade,
//                     double scale, bool tryflip );

// string cascadeName;
// string nestedCascadeName;

// /////////////////////////////////////////////////////////
// // VIP MIX
// void VIP_MIX_Config(void)
// {
//     h2p_vip_mix_addr[0] = 0x00; //stop

//     // din0 is layer 0, background, fixed

//     // layer 2 (log)
//     h2p_vip_mix_addr[2] = 130;
//     h2p_vip_mix_addr[3] = 770;
//     h2p_vip_mix_addr[4] = 0x01;

//     h2p_vip_mix_addr[5] = 0;    //(SCREEN_WIDTH-VIDEO_WIDTH)/2;//layer1 x offset
//     h2p_vip_mix_addr[6] = 0;    //(SCREEN_HEIGHT-VIDEO_HEIGHT)/2;//layer1 y offset
//     h2p_vip_mix_addr[7] = 0x01; //set layer 1 active

//     h2p_vip_mix_addr[0] = 0x01; //start
// }

// /////////////////////////////////////////////////////////
// // VIP Frame Reader: configure
// void VIP_FR_Config(int Width, int Height)
// {
//     int word = Width * Height;
//     int cycle = Width * Height;
//     int interlace = 0;

//     // stop
//     h2p_vip_frame_reader0_addr[0] = 0x00; // stop
//     printf("Width=%d\r\n", Width);
//     printf("Width=%d\r\n", Height);
//     // configure frame 0
//     h2p_vip_frame_reader0_addr[4] = DEMO_VGA_FRAME0_ADDR + FR0_FRAME0_OFFSET; // // frame0 base address
//     h2p_vip_frame_reader0_addr[5] = word;                                     // frame0 word
//     h2p_vip_frame_reader0_addr[6] = cycle;                                    //  The number of single-cycle color patterns to read for the frame
//     h2p_vip_frame_reader0_addr[8] = Width;                                    // frame0 width
//     h2p_vip_frame_reader0_addr[9] = Height;                                   // frame0 height
//     h2p_vip_frame_reader0_addr[10] = interlace;                               // frame0 interlace

//     // configure frame 1
//     h2p_vip_frame_reader0_addr[11] = DEMO_VGA_FRAME0_ADDR + FR0_FRAME1_OFFSET; // // frame1 base address
//     h2p_vip_frame_reader0_addr[12] = word;                                     // frame1 word
//     h2p_vip_frame_reader0_addr[13] = cycle;                                    //  The number of single-cycle color patterns to read for the frame
//     h2p_vip_frame_reader0_addr[15] = Width;                                    // frame1 width
//     h2p_vip_frame_reader0_addr[16] = Height;                                   // frame1 height
//     h2p_vip_frame_reader0_addr[17] = interlace;                                // frame1 interlace

//     h2p_vip_frame_reader0_addr[0] = 0x01; //start

//     // select active frame
//     h2p_vip_frame_reader0_addr[3] = 0; // active frame 0 was set
// }

static int compress_yuyv_to_jpeg(unsigned char *yuyv, FILE *file)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer;
    int z;
    static int written;
    int quality = 90;
    unsigned long size = 0;

    line_buffer = (unsigned char *)calloc(video_out_width * 3, 1);

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, file);
    //dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = video_out_width;
    cinfo.image_height = video_out_heigh;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    z = 0;
    while (cinfo.next_scanline < video_out_heigh)
    {
        int x;
        unsigned char *ptr = line_buffer;

        for (x = 0; x < video_out_width; x++)
        {
            int r, g, b;
            int y, u, v;

            if (!z)
                y = yuyv[0];
            else
                y = yuyv[2];

            u = yuyv[1];
            v = yuyv[3];

            // r = (y + (359 * v)) >> 8;
            // g = (y - (88 * u) - (183 * v)) >> 8;
            // b = (y + (454 * u)) >> 8;
            r = y + 1.13983 * (v - 128);
            g = y - 0.39465 * (u - 128) - 0.58060 * (v - 128);
            b = y + 2.03211 * (u - 128);

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++)
            {
                z = 0;
                yuyv += 4;
            }
        }

        row_pointer[0] = line_buffer;
        //yuyv+= video_out_width*2;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    free(line_buffer);

    return 0;
}

static void yuyv_to_rgb32(int width, int height, unsigned char *src, unsigned long *dst)
{
    unsigned char *s;
    unsigned long *d;
    int l, c, alpha = 0x0;
    int r, g, b, cr, cg, cb, y1, y2;

    l = height;
    s = src;
    d = dst;
    while (l--)
    {
        c = width >> 1;
        while (c--)
        {
            y1 = *s++;
            cb = ((*s - 128) * 454) >> 8;
            cg = (*s++ - 128) * 88;
            y2 = *s++;
            cr = ((*s - 128) * 359) >> 8;
            cg = (cg + (*s++ - 128) * 183) >> 8;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            SAT(r);
            SAT(g);
            SAT(b);

            *dst++ = ((unsigned int)alpha) << 24 | (r << 16) | (g << 8) | b;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            SAT(r);
            SAT(g);
            SAT(b);
            *dst++ = ((unsigned int)alpha) << 24 | (r << 16) | (g << 8) | b;
        }
    }
}

// static int decompress_and_preview(char *file)
// {
//     int x, y;
//     long index;
//     int jpegSubsamp, i, j, location;
//     unsigned long pixel, hindex;
//     unsigned char red, blue, green, alpha;
//     unsigned long size = 0;
//     int width, height;
//     unsigned char *buffer;
//     unsigned char *jbuffer;
//     long screensize = 0;
//     unsigned long *bgr_buff;
//     FILE *jpg_file;
//     int jpg_size = 0;

//      if ((jpg_file = fopen(file, "rb")) ==NULL)
//     {
//         printf("Unable to open JPEG file\n");
//         return -1;
//     }

//     fseek(jpg_file, 0L, SEEK_END);
//     jpg_size = ftell(jpg_file);
//     fseek(jpg_file, 0L, SEEK_SET);
//     jbuffer = (unsigned char *) malloc ( jpg_size * sizeof(unsigned char));

//     fread(jbuffer, jpg_size, 1, jpg_file);

//     size = video_out_heigh * video_out_width;
//     buffer = (unsigned char *) malloc (size * 3 * sizeof(char));
//     bgr_buff = (unsigned long *) malloc (size * sizeof(unsigned long));
//     memset(bgr_buff, 0, (size * sizeof(long)));

//     //fread(buffer, jpg_size, 1, jpg_file);
//     tjhandle _jpegDecompressor = tjInitDecompress();

//     //tjDecompressHeader2(_jpegDecompressor, img, size, &width, &height, &jpegSubsamp);
//     tjDecompressHeader2(_jpegDecompressor, jbuffer, size, &width, &height, &jpegSubsamp);

//     tjDecompress2(_jpegDecompressor, jbuffer, size, buffer, width, 0/*pitch*/, height, TJPF_BGR, TJFLAG_FASTDCT);

//     index=0;
//     j = 0;
//     for (i = 0; i < size; i++) {
//             alpha = 0x30;
//             blue  = buffer[j];
//             green = buffer[j+1];
//             red   = buffer[j+2];
//             /* for ARGB data format */
//             pixel = alpha << 24;
//             pixel |= red << 16;
//             pixel |= green << 8;
//             pixel |= blue;

//             bgr_buff [index] = pixel;
//             index ++;
//             j +=3;
//         }

//    //memcpy(fbp, bgr_buff,  (vinfo.xres * vinfo.yres * vinfo.bits_per_pixel)/8);
//    memcpy((void*)h2p_memory_addr, bgr_buff,  size * sizeof(unsigned long));

//     free(buffer);
//     free(jbuffer);
//     free(bgr_buff);
//     fclose(jpg_file);
//     tjDestroy(_jpegDecompressor);
// }

static int uvc_setup(void)
{
    int ret;
    int videodev_fd;
    struct v4l2_format uvc_v4l2_fmt;
    struct v4l2_requestbuffers uvc_v4l2_requestbuff;
    // fd_set set_of_fd;
    // FD_ZERO(&set_of_fd);
    // FD_SET(videodev_fd, &set_of_fd);
    // struct timeval time_out;
    // time_out.tv_sec = 5;

    // int result = select(videodev_fd + 1, &set_of_fd, NULL, NULL, &time_out);

    // if (result < 0)
    // {
    //     printf("select() error \n");
    //     return -1;
    // }

    // if (result == 0)
    // {
    //     printf("time out \n");
    //     return -1;
    // }

    videodev_fd = open(v4l2_device_file, O_RDWR, 0);
    if (videodev_fd < 0)
    {
        printf("Open video device file failed \n");
        return -1;
    }

    uvc_v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uvc_v4l2_fmt.fmt.pix.width = video_out_width;
    uvc_v4l2_fmt.fmt.pix.height = video_out_heigh;
    uvc_v4l2_fmt.fmt.pix.pixelformat = video_out_fmt;

    if (ioctl(videodev_fd, VIDIOC_S_FMT, &uvc_v4l2_fmt) < 0)
    {
        printf("Set up format failed \n");
        return -1;
    }

    memset(&uvc_v4l2_requestbuff, 0, sizeof(uvc_v4l2_requestbuff));
    uvc_v4l2_requestbuff.count = BUFFER_TEST_NUM;
    uvc_v4l2_requestbuff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uvc_v4l2_requestbuff.memory = V4L2_MEMORY_MMAP;
    if (ioctl(videodev_fd, VIDIOC_REQBUFS, &uvc_v4l2_requestbuff) < 0)
    {
        printf("Request buffer failed \n");
        return -1;
    }
    printf("set up camera ok \n");
    return videodev_fd;
}

static int uvc_start_capturing(int videodev_fd)
{
    int index;
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;

    for (index = 0; index < BUFFER_TEST_NUM; index++)
    {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;

        if (ioctl(videodev_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            printf("Query buffer failed \n");
            return -1;
        }

        buffers[index].length = buf.length;
        buffers[index].offset = buf.m.offset;
        buffers[index].start = (unsigned char *)mmap(NULL, buffers[index].length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                                     videodev_fd, buffers[index].offset);
        memset(buffers[index].start, 0xFF, buffers[index].length);
    }
    printf("mmap ok \n");
    printf("length = %d \n", buf.length);
    for (index = 0; index < BUFFER_TEST_NUM; index++)
    {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;
        buf.m.offset = buffers[index].offset;

        if (ioctl(videodev_fd, VIDIOC_QBUF, &buf) < 0)
        {
            printf("Enqueue buffer failed \n");
            return -1;
        }
    }
    printf("Enqueue ok \n");
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(videodev_fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Stream on failed \n");
        return -1;
    }
    return 0;
}

static int uvc_stop_capturing(int videodev_fd)
{

    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(videodev_fd, VIDIOC_STREAMOFF, &type))
    {
        printf("STOP CAPTURING: Failed \n");
        return -1;
    }
    return 1;
}

static int uvc_capture(int videodev_fd, char *file)
{
    //struct v4l2_buffer buf;
    struct v4l2_buffer buf = {0};
    struct v4l2_format uvc_v4l2_fmt;
    fd_set set_of_fd;
    FD_ZERO(&set_of_fd);
    FD_SET(videodev_fd, &set_of_fd);
    struct timeval time_out;
    time_out.tv_sec = 5;
    //time_out.tv_usec = 0;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    FILE *file_fd = 0;
    file_fd = fopen(file, "wb");

    if (file_fd < 0)
    {
        printf("open file %s failed \n", *(file));
        return -1;
    }

    // if (uvc_start_capturing(videodev_fd) < 0)
    // {
    //     printf("start capturing failed \n");
    //     return -1;
    // }

    uvc_v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(videodev_fd, VIDIOC_G_FMT, &uvc_v4l2_fmt) < 0)
    {
        printf("Get format failed \n");
        return -1;
    }
    int result = select(videodev_fd + 1, &set_of_fd, NULL, NULL, &time_out);

    if (result < 0)
    {
        printf("select() error \n");
        return -1;
    }

    if (result == 0)
    {
        printf("time out \n");
        return -1;
    }

    printf("fd = %d \n", result);

    if (ioctl(videodev_fd, VIDIOC_DQBUF, &buf) < 0)
    {
        printf("Dequeue buffer failed \n");
        return -1;
    }
    //memcpy(thread_data0.frame0, buffer[buf.index].start, sizeof(struct test_buffer));
    compress_yuyv_to_jpeg(buffers[buf.index].start, file_fd);
    //Mat image = Mat(480, 640, CV_8UC3, (void *)buffer[buf.index].start);

    fclose(file_fd);

    // if (ioctl(videodev_fd, VIDIOC_QBUF, &buf) < 0)
    // {
    //     printf("VIDIOC_QBUF failed\n");
    //     return -1;
    // }
    if (uvc_stop_capturing(videodev_fd) < 0)
    {
        printf("stop_capturing failed\n");
        return -1;
    }
    printf("Frame saved in File %s\n", file);
}

static int uvc_streaming(int videodev_fd)
{
    unsigned long *rgb_buffer;
    struct v4l2_buffer buf;
    struct v4l2_format fmt;
    const int nFrameSize = VIDEO_FRAME_MEM_SIZE;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(videodev_fd, VIDIOC_G_FMT, &fmt) < 0)
    {
        printf("Stream test: get video format failed \n");
        return -1;
    }
    rgb_buffer = (unsigned long *)malloc(fmt.fmt.pix.width * fmt.fmt.pix.height * 4);
    if (uvc_start_capturing(videodev_fd) < 0)
    {
        return -1;
    }
    while (1)
    {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(videodev_fd, VIDIOC_DQBUF, &buf) < 0)
        {
            printf("Stream test: Getting frame from memory failed \n");
            break;
        }

        //convert to rgb here
        yuyv_to_rgb32(video_out_width, video_out_heigh, buffers[buf.index].start, rgb_buffer);
        // copy to buffer in user space
        // memcpy((void*)h2p_memory_addr, buffers[buf.index].start,  nFrameSize);

        if (ioctl(videodev_fd, VIDIOC_QBUF, &buf) < 0)
        {
            printf("VIDIOC_QBUF failed\n");
            break;
        }
    }
    if (uvc_stop_capturing(videodev_fd) < 0)
    {
        printf("stop capturing failed \n");
        return -1;
    }
    free(rgb_buffer);
    return 0;
}

// void detectAndDraw( Mat& img, CascadeClassifier& cascade,
//                     CascadeClassifier& nestedCascade,
//                     double scale, bool tryflip )
// {
//     double t = 0;
//     vector<Rect> faces, faces2;
//     const static Scalar colors[] =
//     {
//         Scalar(255,0,0),
//         Scalar(255,128,0),
//         Scalar(255,255,0),
//         Scalar(0,255,0),
//         Scalar(0,128,255),
//         Scalar(0,255,255),
//         Scalar(0,0,255),
//         Scalar(255,0,255)
//     };
//     Mat gray, smallImg;

//     cvtColor( img, gray, COLOR_BGR2GRAY );
//     double fx = 1 / scale;
//     resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR_EXACT );
//     equalizeHist( smallImg, smallImg );

//     t = (double)getTickCount();
//     cascade.detectMultiScale( smallImg, faces,
//         1.1, 2, 0
//         //|CASCADE_FIND_BIGGEST_OBJECT
//         //|CASCADE_DO_ROUGH_SEARCH
//         |CASCADE_SCALE_IMAGE,
//         Size(30, 30) );
//     if( tryflip )
//     {
//         flip(smallImg, smallImg, 1);
//         cascade.detectMultiScale( smallImg, faces2,
//                                  1.1, 2, 0
//                                  //|CASCADE_FIND_BIGGEST_OBJECT
//                                  //|CASCADE_DO_ROUGH_SEARCH
//                                  |CASCADE_SCALE_IMAGE,
//                                  Size(30, 30) );
//         for( vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); ++r )
//         {
//             faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
//         }
//     }
//     t = (double)getTickCount() - t;
//     printf( "detection time = %g ms\n", t*1000/getTickFrequency());
//     for ( size_t i = 0; i < faces.size(); i++ )
//     {
//         Rect r = faces[i];
//         Mat smallImgROI;
//         vector<Rect> nestedObjects;
//         Point center;
//         Scalar color = colors[i%8];
//         int radius;

//         double aspect_ratio = (double)r.width/r.height;
//         if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
//         {
//             center.x = cvRound((r.x + r.width*0.5)*scale);
//             center.y = cvRound((r.y + r.height*0.5)*scale);
//             radius = cvRound((r.width + r.height)*0.25*scale);
//             circle( img, center, radius, color, 3, 8, 0 );
//         }
//         else
//             rectangle( img, Point(cvRound(r.x*scale), cvRound(r.y*scale)),
//                        Point(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
//                        color, 3, 8, 0);
//         if( nestedCascade.empty() )
//             continue;
//         smallImgROI = smallImg( r );
//         nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
//             1.1, 2, 0
//             //|CASCADE_FIND_BIGGEST_OBJECT
//             //|CASCADE_DO_ROUGH_SEARCH
//             //|CASCADE_DO_CANNY_PRUNING
//             |CASCADE_SCALE_IMAGE,
//             Size(30, 30) );
//         for ( size_t j = 0; j < nestedObjects.size(); j++ )
//         {
//             Rect nr = nestedObjects[j];
//             center.x = cvRound((r.x + nr.x + nr.width*0.5)*scale);
//             center.y = cvRound((r.y + nr.y + nr.height*0.5)*scale);
//             radius = cvRound((nr.width + nr.height)*0.25*scale);
//             circle( img, center, radius, color, 3, 8, 0 );
//         }
//     }
//     imshow( "result", img );
// }

int main(int argc, const char **argv)
{

    int fd;
    // int fd_mmap;
    // void *virtual_base;
    // void *axi_virtual_base;
    // //CascadeClassifier cascade, nestedCascade;
    // // double scale;
    // // Mat frame, image;
    // // cv::CommandLineParser parser(argc, argv,
    // //                              "{help h||}"
    // //                              "{cascade|data/haarcascades/haarcascade_frontalface_alt.xml|}"
    // //                              "{nested-cascade|data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|}"
    // //                              "{scale|1|}{try-flip||}{@filename||}");
    // // cascadeName = parser.get<string>("cascade");
    // // nestedCascadeName = parser.get<string>("nested-cascade");
    // // if (!nestedCascade.load(samples::findFileOrKeep(nestedCascadeName)))
    // //     printf("WARNING: Could not load classifier cascade for nested objects \n");
    // // if (!cascade.load(samples::findFile(cascadeName)))
    // // {
    // //     printf("ERROR: Could not load classifier cascade \n");
    // //     return -1;
    // // }
    // fd_mmap = open("/dev/mem", (O_RDWR | O_SYNC));
    // if(fd_mmap < 0) {
    //     printf("ERROR: could not open file /dev/mem \n");
    //     return -1;
    // }
    // virtual_base = mmap(NULL, HW_FPGA_AXI_SPAN, (PROT_READ | PROT_WRITE),MAP_SHARED,fd_mmap, HW_REGS_BASE);
    // if(virtual_base == MAP_FAILED){
    //     printf("ERROR: mmap() failed \n");
    //     return -1;
    // }
    // axi_virtual_base = mmap(NULL, HW_FPGA_AXI_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd_mmap, ALT_AXI_FPGASLVS_OFST);
    // if(axi_virtual_base == MAP_FAILED){
    //     printf("ERROR: axi mmap failed \n");
    //     return -1;
    // }
    // h2p_lw_axi_addr = (unsigned long*)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST) & (unsigned long)(HW_REGS_MASK));
    // h2p_vip_frame_reader0_addr = (unsigned long*)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + ALT_VIP_VFR_0_BASE) & (unsigned long)(HW_REGS_MASK));
    // h2p_memory_addr=(unsigned long*)axi_virtual_base + ( ( unsigned long  )( DEMO_VGA_FRAME0_ADDR) & ( unsigned long)( HW_FPGA_AXI_MASK ) );

    fd = uvc_setup();
    if (fd)
    {
        uvc_start_capturing(fd);
        if (uvc_capture(fd, jpeg_output) < 0)
        {
            printf("failed \n");
            uvc_stop_capturing(fd);
            close(fd);
        }
        else
        {
            printf("show in the screen \n");
            uvc_stop_capturing(fd);

            //decompress_and_preview(jpeg_output);
            close(fd);
            
        }
    }
    // }else{
    //     if(munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
    // 	printf( "ERROR: munmap() failed...\n" );
    // 	close(fd);
    // 	return(-1);
    //     }

    printf("Hello world \n");
    return 0;
}
