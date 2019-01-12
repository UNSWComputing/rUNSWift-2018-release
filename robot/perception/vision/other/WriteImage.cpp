#include "perception/vision/other/WriteImage.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/other/YUV.hpp"


int WriteImage::writeImage(const RegionI& region, write_image_format format, const char* filename, char* title)
{
    // Code adapted from:
    // http://www.labbookpages.co.uk/software/imgProc/files/libPNG/makePNG.c
    
    int width = region.getCols();
    int height = region.getRows();

	int code = 1;
	FILE *fp = NULL;
	png_structp png_ptr = NULL;
	png_infop info_ptr = NULL;
	png_bytep row_rgb = NULL;
	
	// Open file for writing (binary mode)
	fp = fopen(filename, "wb");
	if (fp == NULL) {
		fprintf(stderr, "Could not open file %s for writing\n", filename);
		code = 0;
        // Finalise
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        if (row_rgb != NULL) free(row_rgb);
    
        return code;
	}

	// Initialize write structure
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL) {
		fprintf(stderr, "Could not allocate write struct\n");
		code = 0;
        // Finalise
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        if (row_rgb != NULL) free(row_rgb);
    
        return code;
	}

	// Initialize info structure
	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		fprintf(stderr, "Could not allocate info struct\n");
        code = 0;
        // Finalise
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        if (row_rgb != NULL) free(row_rgb);
    
        return code;	
    }

	// Setup Exception handling
	if (setjmp(png_jmpbuf(png_ptr))) {
		fprintf(stderr, "Error during png creation\n");
        code = 0;
        // Finalise
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        if (row_rgb != NULL) free(row_rgb);
    
        return code;	
    }

	png_init_io(png_ptr, fp);

	// Write header (1 bit colour depth)
	png_set_IHDR(png_ptr, info_ptr, width, height,
			8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	// Set title
	if (title != NULL) {
		png_text title_text;
		title_text.compression = PNG_TEXT_COMPRESSION_NONE;
		title_text.key = (char*) "Title";
		title_text.text = title;
		png_set_text(png_ptr, info_ptr, &title_text, 1);
	}

	png_write_info(png_ptr, info_ptr);

	// Allocate memory for one row (3 bytes per pixel - RGB)
	row_rgb = png_bytep(malloc(3 * width * sizeof(png_byte)));
	
    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;

	if (format == RAW_FORMAT) {
		// Iterator raw
		RegionI::iterator_raw cur_point = region.begin_raw();		
		// RGB data
		RGB rgb;
		uint8_t y, u, v;
		for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
				y = cur_point.getY();
				u = cur_point.getU();
				v = cur_point.getV();
				rgb = yuvToRGB(y, u, v);
				row_rgb[col * 3] = rgb.r;
				row_rgb[col * 3 + 1] = rgb.g;
				row_rgb[col * 3 + 2] = rgb.b;
				++cur_point;
			}
			// Write row
			png_write_row(png_ptr, row_rgb);			
        }
	}
	else if (format == COLOUR_FORMAT) {
		// Iterator fovea
		RegionI::iterator_fovea cur_point = region.begin_fovea();		
		// Write image data
		for(int pixel = 0; pixel < width*height; ++pixel)
		{
			if(cur_point.colour() == 0) {
				// RGB Black
				row_rgb[x * 3] = 0;
				row_rgb[x * 3 + 1] = 0;
				row_rgb[x * 3 + 2] = 0;
			}
			else {
				// RGB White
				row_rgb[x * 3] = 255;
				row_rgb[x * 3 + 1] = 255;
				row_rgb[x * 3 + 2] = 255;            
			}
			++cur_point;
			++x;
			if(x == width)
			{
				// Write row
				png_write_row(png_ptr, row_rgb);
				x = 0;
				++y;
			}
		}
	}
	else {
		std::cout << "Format incorrect\n";
        code = 0;
        // Finalise
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        if (row_rgb != NULL) free(row_rgb);
    
        return code;
	}

	// End write
	png_write_end(png_ptr, NULL);

    // Finalise
	if (fp != NULL) fclose(fp);
	if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
	if (row_rgb != NULL) free(row_rgb);

	return code;
}

RGB WriteImage::yuvToRGB(int y, int u, int v) {
    y -= 16;
    u -= 128;
    v -= 128;

    int r = static_cast<int>((298.082 * y + 0       * u + 408.583 * v) / 256);
    int g = static_cast<int>((298.082 * y - 100.291 * u - 208.120 * v) / 256);
    int b = static_cast<int>((298.082 * y + 516.411 * u + 0       * v) / 256);
	
    // bound each r, g and b value between 0 and 255
    RGB retval;
    retval.r = (r < 0) ? 0 : ((r > 255) ? 255 : r);
    retval.g = (g < 0) ? 0 : ((g > 255) ? 255 : g);
    retval.b = (b < 0) ? 0 : ((b > 255) ? 255 : b);
	
    return retval;
}
