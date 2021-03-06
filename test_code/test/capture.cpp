
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

// #include "opencv2/objdetect.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/videoio.hpp"
extern "C"
{
#include "/opt/libjpeg-turbo/include/jpeglib.h"
//#include <SDL_image.h>
}
//#include <config.h>
// #include "./opencv/include/jerror.h"
// #include "./opencv/include/jmorecfg.h"
#include "/opt/libjpeg-turbo/include/turbojpeg.h"

// #include "opencv2/imgproc/imgproc_c.h"
// #include "opencv2/imgproc/imgproc.hpp"
// #include <opencv2/opencv.hpp>
// #include "opencv2/objdetect.hpp"
#include <iostream>
//#include "hps_0.h"

#define BUFFER_TEST_NUM 4
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

typedef struct
{
    void *start;
    size_t length;
} v4l2_buffer_t;

typedef struct
{
    const char *dev_name;
    int fd;
    int mmap_flag;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers request_buff;
    struct v4l2_buffer buf;

    v4l2_buffer_t *buffer;

    int time_out;

    int pFrame;
} src_v4l2_t;

src_v4l2_t uvc_src_v4l2;
int video_out_width = 1280;
int video_out_heigh = 720;
int video_out_fmt = V4L2_PIX_FMT_YUYV;

char *v4l2_device_file = "/dev/video0";
char *jpeg_output = "Capture.jpg";

static int uvc_get_capability(src_v4l2_t *src_v4l2)
{
    if (ioctl(src_v4l2->fd, VIDIOC_QUERYCAP, &src_v4l2->cap) < 0)
    {
        printf("Device is not support v4l2 \n");
        return -1;
    }

    printf("cap.driver: \"%s\" \n", src_v4l2->cap.driver);
    printf("cap.card: \"%s\"\n", src_v4l2->cap.card);
    printf("cap.bus_info: \"%s\"\n", src_v4l2->cap.bus_info);
    printf("cap.capabilities=0x%08X \n", src_v4l2->cap.capabilities);
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
        printf("- VIDEO_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
        printf("- VIDEO_OUTPUT");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
        printf("- VIDEO_OVERLAY");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VBI_CAPTURE)
        printf("- VBI_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VBI_OUTPUT)
        printf("- VBI_OUTPUT");
    if (src_v4l2->cap.capabilities & V4L2_CAP_RDS_CAPTURE)
        printf("- RDS_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_TUNER)
        printf("- TUNER");
    if (src_v4l2->cap.capabilities & V4L2_CAP_AUDIO)
        printf("- AUDIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_RADIO)
        printf("- RADIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_READWRITE)
        printf("- READWRITE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_ASYNCIO)
        printf("- ASYNCIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_STREAMING)
        printf("- STREAMING");
    if (src_v4l2->cap.capabilities & V4L2_CAP_TIMEPERFRAME)
        printf("- TIMEPERFRAME");

    if (!src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
        printf("device not support capture \n");
        return -1;
    }

    return 0;
}

static int uvc_set_input(src_v4l2_t *src_v4l2)
{
    struct v4l2_input input;
    memset(&input, 0, sizeof(input));
    if (ioctl(src_v4l2->fd, VIDIOC_ENUMINPUT, &input) < 0)
    {
        printf("Unable to query input %d \n", input.index);
        return -1;
    }
    printf("==> Input %i information:", input.index);
    printf("==> name = \"%s\"\n", input.name);
    printf("==> Type = %08X\n", input.type);

    if (ioctl(src_v4l2->fd, VIDIOC_S_INPUT, &input.index) < 0)
    {
        printf("Selecting input failed \n");
        return -1;
    }
    return 0;
}

static int uvc_set_pix_format(src_v4l2_t *src_v4l2)
{
    //struct v4l2_fmtdesc fmt;
    src_v4l2->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    src_v4l2->fmt.fmt.pix.width = video_out_width;
    src_v4l2->fmt.fmt.pix.height = video_out_heigh;
    src_v4l2->fmt.fmt.pix.pixelformat = video_out_fmt;
    if (ioctl(src_v4l2->fd, VIDIOC_S_FMT, &src_v4l2->fmt) < 0)
    {
        printf("Setting format failed \n");
        return -1;
    }else{
        printf("==> Setting format: (%d, %d)\n", video_out_width, video_out_heigh);
    }
    return 0;
}

int uvc_set_fps(src_v4l2_t *src_v4l2)
{
    struct v4l2_streamparm setfps;

    memset(&setfps, 0, sizeof(setfps));

    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 1;
    //setfps.parm.capture.timeperframe.denominator = src_v4l2->fps;
    if (ioctl(src_v4l2->fd, VIDIOC_S_PARM, &setfps) == -1)
    {
        /* Not fatal - just warn about it */
        printf("Error setting frame rate:");
        printf("VIDIOC_S_PARM: %s", strerror(errno));
        return (-1);
    }else{
        printf("'==> Set stream parameter successfully\n");
    }

    return (0);
}

static int uvc_free_mmap(src_v4l2_t *src)
{
    int i;
    for (i = 0; i < src->request_buff.count; i++)
        munmap(src->buffer[i].start, src->buffer[i].length);

    return (0);
}

static int uvc_set_mmap(src_v4l2_t *src)
{
    enum v4l2_buf_type type;
    int index;
    struct v4l2_buffer buf;
    if (~src->cap.capabilities & V4L2_CAP_STREAMING)
    {
        printf("device not support capturing \n");
        return -1;
    }

    memset(&src->request_buff, 0, sizeof(src->request_buff));
    src->request_buff.count = 4;
    src->request_buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    src->request_buff.memory = V4L2_MEMORY_MMAP;

    if (ioctl(src->fd, VIDIOC_REQBUFS, &src->request_buff) < 0)
    {
        printf("Error requesting buffer for mmemory map \n");
        return -1;
    }

    printf("frames = %d \n", src->request_buff.count);
    if (src->request_buff.count < 2)
    {
        printf("Insufficient memory \n");
        return -1;
    }

    src->buffer = (v4l2_buffer_t *)calloc(src->request_buff.count, sizeof(v4l2_buffer_t));
    if (src->buffer == NULL)
    {
        printf("Can not allocate for memory \n");
        return -1;
    }

    for (index = 0; index < src->request_buff.count; index++)
    {
        
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;

        if (ioctl(src->fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            printf("Queue buffer failed \n");
            return -1;
        }

        src->buffer[index].length = buf.length;
        src->buffer[index].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, src->fd,
                                        buf.m.offset);
        if (src->buffer[index].start == MAP_FAILED)
        {
            printf("Mapping memory %d failed \n", index);
            src->request_buff.count = index;
            uvc_free_mmap(src);
            free(src->buffer);
            src->buffer = NULL;
            return -1;
        }
        printf("%d length = %d \n", index, src->buffer[index].length);
    }

    for (index = 0; index < src->request_buff.count; index++)
    {
        memset(&src->buf, 0, sizeof(src->buf));
        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;
        src->buf.index = index;
        if (ioctl(src->fd, VIDIOC_QBUF, &src->buf) == -1)
        {
            printf("VIDIOC_QBUF: %s \n", strerror(errno));
            uvc_free_mmap(src);
            free(src->buffer);
            src->buffer = NULL;
            return (-1);
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(src->fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Start capturing failed \n");
        uvc_free_mmap(src);
        free(src->buffer);
        src->buffer = NULL;
        return -1;
    }
    return 0;
}


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

static int uvc_capture(src_v4l2_t *src, const char* file)
{
    printf("start capturing \n");
    if(src->time_out)
    {
        fd_set fds;
        struct timeval tv;
        int result;

        FD_ZERO(&fds);
        FD_SET(src->fd, &fds);

        tv.tv_sec = src->time_out;
        tv.tv_usec = 0;

        result = select((src->fd) + 1, &fds, NULL, NULL, &tv);

        if(result == -1)
        {
            printf("Select() function failed \n");
            return -1;
        }
        if(result == 0)
        {
            printf("Time out \n");
            return -1;
        }
    }

    if(src->mmap_flag == 1)
    {
        printf("mmap in processing \n");
        FILE* file_fd;
        file_fd = fopen(file, "wb");
        if(file_fd == NULL)
        {
            printf("open file %s failed \n", file);
            return -1;
        }
        if(src->pFrame > 0)
        {
            if(ioctl(src->fd, VIDIOC_QBUF, &src->buf) < 0)
            {
                printf("VIDIOC_QBUF: %s \n", strerror(errno));
                return -1;
            }
        }

        memset(&src->buf, 0, sizeof(src->buf));

        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;

        if(ioctl(src->fd, VIDIOC_DQBUF, &src->buf) < 0)
        {
            printf("VIDIOC_DQBUF: %s \n", strerror(errno));
        }
        else{
            printf("==>Dequeue buffer successfully\n");
        }
        compress_yuyv_to_jpeg((unsigned char*)src->buffer[src->buf.index].start, file_fd);
        fclose(file_fd);
        printf("Frame saved in File %s\n", file);
    }
}

static int uvc_open(src_v4l2_t *src)
{
    int fd_ret;
    src->fd = open(src->dev_name, O_RDWR, 0);
    if(src->fd < 0)
    {
        printf("Open device %s failed \n", *(src->dev_name));
        return -1;
    }
    src->pFrame = -1;
    fd_ret = src->fd;
    return fd_ret;
}

static int uvc_close(src_v4l2_t *src)
{
    if(src->buffer != NULL)
    {
        uvc_free_mmap(src);
        free(src->buffer);
    }
    close(src->fd);
    return 0;
}

static void srcv4l2_init(src_v4l2_t *src)
{
    src->dev_name = v4l2_device_file;
    src->mmap_flag = 1;
    src->time_out = 5;
}


int main()
{

    srcv4l2_init(&uvc_src_v4l2);
    uvc_open(&uvc_src_v4l2);
    uvc_get_capability(&uvc_src_v4l2);
    uvc_set_input(&uvc_src_v4l2);
    uvc_set_pix_format(&uvc_src_v4l2);
    //uvc_set_fps(&uvc_src_v4l2);
    uvc_set_mmap(&uvc_src_v4l2);
    uvc_capture(&uvc_src_v4l2, jpeg_output);
    uvc_close(&uvc_src_v4l2);

    return 0;
}
