#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdint.h>


#include <unistd.h>
#include <termios.h>

#define USE_RGB 1
#define USE_FILTER 1


#define CAM_WIDTH 680
#define CAM_HEIGHT 576
#ifdef USE_RGB
#define CAM_FORMAT V4L2_PIX_FMT_RGB565 
#else
#define CAM_FORMAT V4L2_PIX_FMT_YVYU
// (v4l2_fourcc('B','G','R','A'))
#endif




char getch() {
    char buf = 0;
    struct termios old = { 0 };
    if (!isatty(0))
        return 0;
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 0;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return (buf);
}

/**
 * BITMAPFILEHEADER
 *
 * The BITMAPFILEHEADER structure contains information about the type, size,
 * and layout of a file that contains a DIB [device-independent bitmap].
 *
 * Adapted from http://msdn.microsoft.com/en-us/library/dd183374(VS.85).aspx.
 */
typedef struct {
    uint16_t   bfType;
    uint32_t  bfSize;
    uint16_t   bfReserved1;
    uint16_t   bfReserved2;
    uint32_t  bfOffBits;
} __attribute__((__packed__))
BITMAPFILEHEADER;

/**
 * BITMAPINFOHEADER
 *
 * The BITMAPINFOHEADER structure contains information about the
 * dimensions and color format of a DIB [device-independent bitmap].
 *
 * Adapted from http://msdn.microsoft.com/en-us/library/dd183376(VS.85).aspx.
 */
typedef struct {
    uint32_t  biSize;
    int32_t   biWidth;
    int32_t   biHeight;
    uint16_t   biPlanes;
    uint16_t   biBitCount;
    uint32_t  biCompression;
    uint32_t  biSizeImage;
    int32_t   biXPelsPerMeter;
    int32_t   biYPelsPerMeter;
    uint32_t  biClrUsed;
    uint32_t  biClrImportant;
} __attribute__((__packed__))
BITMAPINFOHEADER;

static const uint32_t bitfields[4] = {
    0x0000F800, // red
    0x000007E0, // green
    0x0000001F, // blue 
    0 // reserved
};

int savebitmap(int bytesperline, int width, int height, u_int8_t* data) {
    FILE* fp;
    uint32_t bytes_per_row = (width * 2 + 3) & ~3;
    uint32_t image_size = bytes_per_row * height;
    uint32_t header_size = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + sizeof(bitfields);
    uint32_t data_size = header_size + image_size;
    char filename[256] = "output.bmp";

    static int count;
    sprintf(filename, "output-%02d.bmp", count);
    count++;

    // Assumes low endian
    BITMAPFILEHEADER fileHeader = {
       .bfType = 0x4d42, // 'B' 'M'
       .bfSize = data_size,
       .bfReserved1 = 0,
       .bfReserved2 = 0,
       .bfOffBits = header_size
    };

    BITMAPINFOHEADER infoHeader = {
       .biSize = sizeof(BITMAPINFOHEADER) + sizeof(bitfields),
       .biWidth = width,
       .biHeight = -height,
       .biPlanes = 1,
       .biBitCount = 16,
       .biCompression = 3,
       .biSizeImage = image_size,
       .biXPelsPerMeter = 0,
       .biYPelsPerMeter = 0,
       .biClrUsed = 0,
       .biClrImportant = 0,
    };

    printf("Writing '%s'\n", filename);
    fp = fopen(filename, "wb");
    if (fp == NULL) {
        printf("Failed to open file\n");
        return -1;
    }

    if (fwrite(&fileHeader, sizeof(fileHeader), 1, fp) != 1
        || fwrite(&infoHeader, sizeof(infoHeader), 1, fp) != 1
        || fwrite(&bitfields, sizeof(bitfields), 1, fp) != 1) {
        printf("Failed to write image\n");
        return -1;
    }
    for (int i = 0; i < height; i++) {
        u_int16_t* src = data + bytesperline * i;
        if (fwrite(src, width * 2, 1, fp) != 1) {
            printf("Failed to write image\n");
            return -1;
        }
    }


    fclose(fp);
    return 0;
}

static inline uint16_t median_of_five(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e) {
    return b < a ? d < c ? b < d ? a < e ? a < d ? e < d ? e : d
        : c < a ? c : a
        : e < d ? a < d ? a : d
        : c < e ? c : e
        : c < e ? b < c ? a < c ? a : c
        : e < b ? e : b
        : b < e ? a < e ? a : e
        : c < b ? c : b
        : b < c ? a < e ? a < c ? e < c ? e : c
        : d < a ? d : a
        : e < c ? a < c ? a : c
        : d < e ? d : e
        : d < e ? b < d ? a < d ? a : d
        : e < b ? e : b
        : b < e ? a < e ? a : e
        : d < b ? d : b
        : d < c ? a < d ? b < e ? b < d ? e < d ? e : d
        : c < b ? c : b
        : e < d ? b < d ? b : d
        : c < e ? c : e
        : c < e ? a < c ? b < c ? b : c
        : e < a ? e : a
        : a < e ? b < e ? b : e
        : c < a ? c : a
        : a < c ? b < e ? b < c ? e < c ? e : c
        : d < b ? d : b
        : e < c ? b < c ? b : c
        : d < e ? d : e
        : d < e ? a < d ? b < d ? b : d
        : e < a ? e : a
        : a < e ? b < e ? b : e
        : d < a ? d : a;
}

#ifdef USE_RGB

static uint16_t lookup[65536];

uint16_t colors_rgb[16] = {
    0x0000,
    0xffff,
    0xa1c9,
    0x8719,

    0xa1f6,
    0x5deb,
    0x3998,
    0xf7f0,

    0xa2e4,
    0x6a20,
    0xdb90,
    0x5acb,

    0x8c71,
    0xb7f7,
    0x83bf,
    0xc638,
};

static void init_lookup() {
    // split colors into components
    u_int8_t rc[16],gc[16],bc[16];
    for (int c = 0; c < 16; c++) {
        rc[c] = (colors_rgb[c] >> 11) & 0x1f;
        gc[c] = (colors_rgb[c] >> 5) & 0x3f;
        bc[c] = (colors_rgb[c] >> 0) & 0x1f;
    }
    for (int r = 0; r < 32; r++) {
        for (int g = 0; g < 64; g++) {
            for (int b = 0; b < 32; b++) {
                int idx = (r<<11)|(g<<5)|(b<<0);
    #if USE_FILTER
                int max = 0x1000000;
    #define POW(x) ((x)*(x))
                lookup[idx] = idx;
                for (int i = 0; i < 16; i++) {
                    int dist = POW(r - rc[i]) + POW(g - gc[i]) + POW(b - bc[i]);
                    if (dist < 30 && dist < max) {
                        max = dist;
                        lookup[idx] = colors_rgb[i];
                    }
                }
    #else
                lookup[idx] = idx;
    #endif
            }
        }
    }
}

#else //YUV

static uint16_t lookup[256][256][256];


struct yuv {
    uint8_t y, u, v;
} colors_yuv[16] = {
    { 0x9a, 0x7b, 0xcc},// 0 black         9a7b 9acb 997b 9acc 9b7c 9bcc 9a7d 9bcb
    { 0xfe, 0x80, 0x80},// 1 white         fe80 fe80 fe80 fe80 fe80 fe80 fe80 fe80
    { 0x67, 0xb7, 0x79},// 2 red           67b7 6879 67b7 6779 66b5 6579 67b6 6579
    { 0xd1, 0x52, 0x87},// 3 cyan          d151 ce88 cf52 d087 d152 d287 d252 d088
    { 0x71, 0xad, 0xac},// 4 purple        79ad 7aad 79ac 7aad 7cac 7aad 7aac 79ad
    { 0xa8, 0x54, 0x61},// 5 green         a855 a861 a755 aa61 a754 a661 a754 a661
    { 0x53, 0x7c, 0xcc},// 6 blue          527c 4fcc 507c 53cc 537c 54cc 537b 51cc
    { 0xfe, 0x85, 0x40},// 7 yellow        fe85 fe40 fe85 fe40 fe84 fe3f fc84 fc40
    { 0x7c, 0xaa, 0x56},// 8 orange        7baa 7a57 7caa 7d56 7daa 7d56 79aa 7956
    { 0x53, 0x9e, 0x47},// 9 brown         559e 5447 529e 5247 529e 5247 539e 5348
    { 0xa6, 0xb5, 0x78},// 10 light red    a5b5 a678 a6b5 a978 aab5 a978 a7b5 a678
    { 0x65, 0x80, 0x80},// 11 dark grey    6580 6680 6580 6580 6680 6580 6680 6580
    { 0x97, 0x80, 0x80},// 12 grey         9780 9780 9780 9780 9880 9780 9780 9780
    { 0xfe, 0x54, 0x60},// 13 light green  fe55 fe60 fe54 fe61 fe54 fd60 fd54 fd61
    { 0x9a, 0x7d, 0xcc},// 14 light blue   9a7d 98cd 987d 9acc 9b7d 9bcc 9c7d 9acc
    { 0xcf, 0x80, 0x80},// 15 light grey   cf80 ce80 ce80 cf80 d080 cf80 d080 cf80
};

static uint16_t convert_yuv_to_rgb_pixel(u_int8_t y, u_int8_t u, u_int8_t v) {
    int r, g, b;
    /*
    r = y + (1.370705 * (v-128));
    g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
    b = y + (1.732446 * (u-128));
    */
    int C = y - 16;
    int D = u - 128;
    int E = v - 128;
    r = ((298 * C + 409 * E + 128) >> 8);
    g = ((298 * C - 100 * D - 208 * E + 128) >> 8);
    b = ((298 * C + 516 * D + 128) >> 8);

    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | ((b & 0xf8) >> 3);
}

static void init_lookup() {
    for (int y = 0; y < 256; y++) {
        for (int u = 0; u < 256; u++) {
            for (int v = 0; v < 256; v++) {
#if USE_FILTER
                int max = 0x1000000;
#define POW(x) ((x)*(x))
                for (int i = 0; i < 16; i++) {
                    int dist = POW(y - colors[i].y) + POW(u - colors[i].u) + POW(v - colors[i].v);
                    if (dist < max) {
                        max = dist;
                        lookup[y][u][v] = convert_yuv_to_rgb_pixel(colors[i].y, colors[i].u, colors[i].v);
                    }
                }
#else
                lookup[y][u][v] = convert_yuv_to_rgb_pixel(y, u, v);
#endif
            }
        }
    }
}
#endif

#define CAM_BUFFERS 1

struct decoder {
    int cam_fd;
    uint8_t* cam_buffers[6];
    int cam_buffer_index;
    int cam_width;
    int cam_height;
    int cam_bytesperline;

    uint8_t* fb_p;
    int fb_width;
    int fb_height;
    int fb_bytesperline;
};

struct decoder* fb_init(struct decoder* dec) {
    int fb_fd;
    uint8_t* fb_p;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    /* Setting framebuffer */
    fb_fd = open("/dev/fb0", O_RDWR);
    if (!fb_fd) {
        fprintf(stderr, "%s:%i: Unable to open framebuffer\n", __FILE__, __LINE__);
        return 0;
    }
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo);
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        fprintf(stderr, "%s:%i: Unable to get framebuffer info\n", __FILE__, __LINE__);
        return 0;
    }
    dec->fb_width = vinfo.xres;
    dec->fb_height = vinfo.yres;
    dec->fb_bytesperline = vinfo.xres * vinfo.bits_per_pixel / 8;
    printf("Framebuffer: resolution %dx%d with %dbpp\n\r", dec->fb_width, dec->fb_height, vinfo.bits_per_pixel);

    fb_p = (uint8_t*)mmap(0, dec->fb_width * dec->fb_height * 4, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);

    memset(fb_p, 0, dec->fb_width * dec->fb_height * 4);
    dec->fb_p = fb_p;
    return dec;
}

struct decoder* cam_init(struct decoder* dec) {
    struct v4l2_requestbuffers reqbuf;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buffinfo;
    enum v4l2_buf_type bufftype;
    //int cam_buffer_size;
    int cam_fd;

    /* Setting camera */
    cam_fd = open("/dev/video0", O_RDWR | O_NONBLOCK, 0);
    if (!cam_fd) {
        fprintf(stderr, "%s:%i: Couldn't open device\n", __FILE__, __LINE__);
        return 0;
    }
    if (ioctl(cam_fd, VIDIOC_QUERYCAP, &cap)) {
        fprintf(stderr, "%s:%i: Couldn't retreive device capabilities\n", __FILE__, __LINE__);
        return 0;
    }
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        fprintf(stderr, "%s:%i: Device is not a capture device\n", __FILE__, __LINE__);
        return 0;
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == 0) {
        fprintf(stderr, "%s:%i: Device is not available for streaming", __FILE__, __LINE__);
        return 0;
    }

    /* Set image format */
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CAM_WIDTH;
    fmt.fmt.pix.height = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = CAM_FORMAT;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) == -1) {
        fprintf(stderr, "%s:%i: Unable to set image format\n", __FILE__, __LINE__);
        return 0;
    }
    //cam_buffer_size = fmt.fmt.pix.sizeimage;
    dec->cam_fd = cam_fd;
    dec->cam_width = fmt.fmt.pix.width;
    dec->cam_height = fmt.fmt.pix.height;
    dec->cam_bytesperline = fmt.fmt.pix.bytesperline;

    /* Request buffers */
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count = CAM_BUFFERS;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &reqbuf) == -1) {
        fprintf(stderr, "%s:%i: Mmap streaming not supported\n", __FILE__, __LINE__);
        return 0;
    }
    if (reqbuf.count < CAM_BUFFERS) {
        fprintf(stderr, "%s:%i: Not all requared buffers are allocated\n", __FILE__, __LINE__);
        return 0;
    }
    printf("Camera: resolution %dx%d with %dbpp buffer count %d\n\r",
        dec->cam_width, dec->cam_height,
        8 * dec->cam_bytesperline / dec->cam_width, reqbuf.count);

    /* Query and Mmap buffers */
    for (int i = 0; i < reqbuf.count; i++) {
        memset(&buffinfo, 0, sizeof(buffinfo));
        buffinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffinfo.memory = V4L2_MEMORY_MMAP;
        buffinfo.index = i;

        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &buffinfo) == -1) {
            fprintf(stderr, "%s:%i: Unable to query buffers\n", __FILE__, __LINE__);
            return 0;
        }

        dec->cam_buffers[i] = mmap(NULL, buffinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buffinfo.m.offset);

        if (dec->cam_buffers[i] == MAP_FAILED) {
            fprintf(stderr, "%s:%i: Unable to enqueue buffers\n", __FILE__, __LINE__);
            return 0;
        }
    }

    /* Enqueue buffers */
    for (int i = 0; i < reqbuf.count; i++) {
        memset(&buffinfo, 0, sizeof(buffinfo));
        buffinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffinfo.memory = V4L2_MEMORY_MMAP;
        buffinfo.index = i;

        if (ioctl(cam_fd, VIDIOC_QBUF, &buffinfo) == -1) {
            fprintf(stderr, "%s:%i: Unable to enqueue buffers\n", __FILE__, __LINE__);
            return 0;
        }
    }

    /* Start Streaming */
    bufftype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &bufftype) == -1) {
        fprintf(stderr, "%s:%i: Unable to start streaming\n", __FILE__, __LINE__);
        return 0;
    }

    return dec;
}

static void show(struct decoder* dec, uint8_t* cam_buffer) {
    static int c;

    //memcpy(fb_p, cam_buffer, dec->fb_width*dec->fb_height*4);

    for (int i = 0; i < dec->cam_height; i++) {
        u_int16_t* src = cam_buffer + dec->cam_bytesperline * i;
        u_int16_t* dst = dec->fb_p + dec->fb_width * (i * 2 + (c & 1))* 2;
#ifdef USE_RGB
#if USE_FILTER
#if DOUBLE
        // use median
        u_int16_t* dst1 = dec->fb_p + (i * 4 + (c & 1) * 2) * dec->fb_width * 2;
        u_int16_t* dst2 = dec->fb_p + (i * 4 + (c & 1) * 2 + 1) * dec->fb_width * 2;
        u_int16_t* src = cam_buffer + dec->cam_bytesperline * i;
        src++;src++;src++;
        for (int j = 2; j < dec->cam_width - 3; j++) {
            u_int16_t d = median_of_five(src[-2], src[1], src[0], src[1], src[2]);
            uint8_t r = d >> 11;
            uint8_t g = (d >> 5) & 0x3f;
            uint8_t b = d & 0x1f;
            //d = ((r & 0x18) << 11) | ((g & 0x38) << 5) | (b & 0x18);
            src[0] = d;
            *dst1 = d;
            dst1++;
            *dst2 = d;
            dst2++;
            *dst1 = d;
            dst1++;
            *dst2 = d;
            dst2++;
            src++;
        }
#else
        for (int j = 0; j < dec->cam_width; j++) {
            u_int16_t d = lookup[src[j]];
            src[j] = d;
            dst[j] = d;
        }
        //memcpy(dst, src, dec->cam_bytesperline);
#endif
#else
        memcpy(dst, src, dec->cam_bytesperline);
#endif

#else // YUV
#if DOUBLE
        u_int16_t* dst1 = (uint16_t*)dec->fb_p + (i * 4 + (c & 1) * 2) * dec->fb_width;
        u_int16_t* dst2 = (uint16_t*)dec->fb_p + (i * 4 + (c & 1) * 2 + 1) * dec->fb_width;
        u_int8_t* data = cam_buffer + dec->cam_bytesperline * i;
        for (int j = 0; j < dec->cam_width * 2; j += 4) {

            u_int8_t y0 = data[j + 1];
            u_int8_t u = data[j + 0];
            u_int8_t y1 = data[j + 3];
            u_int8_t v = data[j + 2];

            //uint16_t pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);
            uint16_t pixel32 = lookup[y0][u][v];

            *dst2 = *dst1 = pixel32;
            dst1++;dst2++;
            *dst2 = *dst1 = pixel32;
            dst1++;dst2++;
            //pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);
            pixel32 = lookup[y1][u][v];
            *dst2 = *dst1 = pixel32;
            dst1++;dst2++;
            *dst2 = *dst1 = pixel32;
            dst1++;dst2++;
        }
#else
u_int16_t* dst = (uint16_t*)dec->fb_p + (i * 2 + (c & 1)) * dec->fb_width;
u_int8_t* data = cam_buffer + dec->cam_bytesperline * i;
for (int j = 0; j < dec->cam_width * 2; j += 4) {

    u_int8_t y0 = data[j + 1];
    u_int8_t u = data[j + 0];
    u_int8_t y1 = data[j + 3];
    u_int8_t v = data[j + 2];

    //uint16_t pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);
    uint16_t pixel32 = lookup[y0][u][v];

    *dst = pixel32;
    dst++;
    //pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);
    pixel32 = lookup[y1][u][v];
    *dst= pixel32;
    dst++;
}
#endif
#endif
    }
    c++;

}

int read_frame(struct decoder* dec) {
    while (1) {
        struct v4l2_buffer buffinfo;
        fd_set fds;
        struct timeval tv;
        int r;
    
    
        FD_ZERO(&fds);
        FD_SET(dec->cam_fd, &fds);
        tv.tv_sec = 2;
        tv.tv_usec = 0;
    
        r = select(dec->cam_fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno = EINTR)
                continue;
            fprintf(stderr, "%s:%i: Call to select() failed\n", __FILE__, __LINE__);
            return -1;
        }
        if (r == 0) {
            fprintf(stderr, "%s:%i: Call to select() timeout\n", __FILE__, __LINE__);
            continue;
        }
    
        if (!FD_ISSET(dec->cam_fd, &fds))
            continue;
    
        memset(&buffinfo, 0, sizeof(buffinfo));
        buffinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffinfo.memory = V4L2_MEMORY_MMAP;
        if (ioctl(dec->cam_fd, VIDIOC_DQBUF, &buffinfo) == -1) {
            if (errno == EAGAIN)
                continue;
            fprintf(stderr, "%s:%i: Unable to dequeue buffer\n", __FILE__, __LINE__);
            return -1;
        }
    
        dec->cam_buffer_index = buffinfo.index;
        return 0;
    }    
}

int free_frame(struct decoder* dec) {
    struct v4l2_buffer buffinfo;
    memset(&buffinfo, 0, sizeof(buffinfo));
    buffinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffinfo.memory = V4L2_MEMORY_MMAP;
    buffinfo.index = dec->cam_buffer_index;
    if (ioctl(dec->cam_fd, VIDIOC_QBUF, &buffinfo) == -1) {
        fprintf(stderr, "%s:%i: Unable to enqueue buffer\n", __FILE__, __LINE__);
        return -1;
    }
    return 0;
}

int main(void) {

    struct decoder* dec = calloc(1, sizeof(*dec));

    if (!fb_init(dec)) {
        return -1;
    }
    if (!cam_init(dec)) {
        return -1;
    }

    init_lookup();

    unsigned long last_t, t, count = 0;
    while (1) {
        if (read_frame(dec)) {
            return -1;
        }
        show(dec, dec->cam_buffers[dec->cam_buffer_index]);

        if (getch() == ' ') {
            savebitmap(dec->cam_bytesperline, dec->cam_width, dec->cam_height, dec->cam_buffers[dec->cam_buffer_index]);
        }
        free_frame(dec);

        t = time(0);
        if (t != last_t) {
            printf("Count %d\n", (int)count);
            count = 0;
            last_t = t;
        }
        count++;
    }

    return 0;
}
