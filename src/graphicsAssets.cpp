
#include <graphicsAssets.h>
#include <graphicsGlobals.h>

void imageWrapper(FrameBuffer bbuf) {
  gAsset.image(bbuf);
}

//###################################################################
// Fill the backbuffer in preperation for the next displayed frame
// Assumes a 120x48 image imported from Gimp in rgb565 format (two
// bytes per pixel).  Images stored in include/images.h
//###################################################################
void GraphicsAssets::image(FrameBuffer bbuf) {

  // Reading an image created by Gimp (loaded from images.h)
  // and expanding/writing the bytes into the framebuffer.  framebuffer always
  // is loaded with rgb888  (one byte per r, g, b) per pixel.
  for (unsigned col = 0; col < imageTable[imageToDisplayIndex]->width; col++) {

    // Start at row 0, this column in the Gimp pixel array
    const uint8_t* p = imageTable[imageToDisplayIndex]->pixel_data + (col * 2);
    for (unsigned row = 0; row < imageTable[imageToDisplayIndex]->height; row++) {
      #if GIMP_RGB565_LITTLE_ENDIAN
        uint16_t rgb565 = (p[1] << 8) | p[0];
      #else
        uint16_t rgb565 = (p[0] << 8) | p[1];
      #endif

      // Read each of the sub-fields to extract rgb from the Gimp 2-byte data
      uint8_t r5 = (rgb565 >> 11) & 0x1F;
      uint8_t g6 = (rgb565 >> 5)  & 0x3F;
      uint8_t b5 =  rgb565        & 0x1F;

      // Expand 5/6-bit channels to full 8-bit range (0–255)
      uint8_t r8 = (r5 << 3) | (r5 >> 2);
      uint8_t g8 = (g6 << 2) | (g6 >> 4);
      uint8_t b8 = (b5 << 3) | (b5 >> 2);

      // Now store the rgb (three bytes per row) data into the backbuffer.  In the rendering (by core-1), 
      // we will add the start bytes, brightness, gamma, and stop bytes before DMA to the dotStars.
      // Reverse the rows since Gimp stores row 0 at the top and our framebuffer starts with 0 at the bottom.
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3)] = r8;
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 1)] = g8;
      bbuf[((ROWS-1-row) * COLUMNS * 3) + (col * 3 + 2)] = b8;

      // Advance to next row, same column
      p += imageTable[imageToDisplayIndex]->width * 2;
    }
  }
}