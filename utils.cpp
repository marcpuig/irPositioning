#include <time.h>
#include <png.h>
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */

#include "defines.h"
#include "utils.h"

historyUnitT history[HIST_SIZE];
int lastHistoryPos = -1;
int historyLength = 0;
extern int64_t startSec;
#ifdef THUMBNAILS
extern unsigned char imageDataThumbnail[PIXELS_X/8 * PIXELS_Y/8];
#endif

void abort_(const char * s, ...) {
    va_list args;
    va_start(args, s);
    vfprintf(stderr, s, args);
    fprintf(stderr, "\n");
    va_end(args);
    abort();
}

// Output [0 -> 360)
float correctAngle(float angle) {
    if (angle >= 360)
        angle = fmod(angle, 360);
    else if (angle <= -360) {
        angle = fmod(angle, 360);
        angle += 360; 
    }
    else if (angle < 0)
        angle += 360; 
    
    return angle;
}

// Output (-180 -> 180]
float correctAngleOffset(float angle) {
    if (angle >= 360 || angle <= -360)
        angle = fmod(angle, 360);
    
    if (angle > 180)
        angle -= 360;
    else if (angle <= -180)
        angle += 360; 
    
    return angle;
}

Chrono::Chrono() {
    clock_gettime(CLOCK_REALTIME, &t1);
}

long int Chrono::click() {
    clock_gettime(CLOCK_REALTIME, &t2);
    return getSpent((t2.tv_nsec - t1.tv_nsec) / 1000);
}

int64_t getSec() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec;
}

long int getMillisSinceStart() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_nsec / 1000000 + (t.tv_sec - startSec) * 1000;
}

long int getMicroSec() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_nsec / 1000;
}

long int getSpent(long int dif) {
    if (dif < 0)
        dif += 1000000;
    return dif;
}

#ifdef THUMBNAILS
void write_png_file(int photogram) {
    char file_name[100];
    png_infop info_ptr = NULL;
    png_structp png_ptr = NULL;
    static png_byte ** row_pointers = NULL;
    static bool initialized = false;
    
    sprintf(file_name, "photograms/%6d.png", photogram);
    
    /* create file */
    FILE *fp = fopen(file_name, "wb");
    if (!fp)
        abort_("[write_png_file] File %s could not be opened for writing", file_name);


    /* initialize stuff */
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
        abort_("[write_png_file] png_create_write_struct failed");

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        abort_("[write_png_file] png_create_info_struct failed");

    /* write header */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during writing header");

    png_set_IHDR(png_ptr,
                info_ptr,
                PIXELS_X/8,
                PIXELS_Y/8,
                8,
                PNG_COLOR_TYPE_GRAY,
                PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_BASE,
                PNG_FILTER_TYPE_BASE);

     
    if (!initialized) {
        /* Initialize rows of PNG. */
        row_pointers = (png_byte**)png_malloc (png_ptr, PIXELS_Y/8 * sizeof (png_byte *));
        for (int y = 0; y < PIXELS_Y/8; ++y)
            row_pointers[y] = imageDataThumbnail + y * PIXELS_X/8;

        initialized = true;
    }
    
    /* Write the image data to "fp". */
    png_init_io (png_ptr, fp);
    png_set_rows (png_ptr, info_ptr, row_pointers);
    png_write_png (png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

    free(png_ptr);
    
    fclose(fp);
}
#endif