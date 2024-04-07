// This code is an OpenCL kernel for applying a perspective warp transformation to an image, 
// interpolating pixel values based on a given transformation matrix.
#if 0
워프 변환(warp transformation)은 이미지의 픽셀 위치를 재배치하여 이미지의 모양을 변형시키는 과정입니다. 
이 변환을 통해 이미지에 대한 시점 변경, 물체의 변형, 혹은 다양한 효과를 적용할 수 있습니다. 
특히, 워프 변환 중 하나인 투시 워프 변환(perspective warp transformation)은 
이미지에 3차원적인 효과를 주어 물체가 공간에서 어떻게 위치하고 있는지를 변형시키는 데 사용됩니다. 
이를 통해 이미지를 마치 다른 각도에서 본 것처럼 만들거나, 물체를 다른 모양으로 변형시키는 등의 효과를 줄 수 있습니다. 
예를 들어, 건물의 사진을 마치 공중에서 내려다보는 시점으로 변형시키거나, 도로의 사진을 더욱 깊이 있게 보이게 하는 등의 작업이 가능합니다.
#endif
// Define constants for interpolation bits, table size, and scale factor
#define INTER_BITS 5
#define INTER_TAB_SIZE (1 << INTER_BITS)
#define INTER_SCALE 1.f / INTER_TAB_SIZE

// Define constants for remapping coefficients' bits and scale factor
#define INTER_REMAP_COEF_BITS 15
#define INTER_REMAP_COEF_SCALE (1 << INTER_REMAP_COEF_BITS)

// Define the kernel function for performing a warp perspective transformation
__kernel void warpPerspective(__global const uchar * src,
                              int src_row_stride, int src_px_stride, int src_offset, int src_rows, int src_cols,
                              __global uchar * dst,
                              int dst_row_stride, int dst_offset, int dst_rows, int dst_cols,
                              __constant float * M)
{
    // Get the global IDs for the current thread to determine the pixel position in the destination image
    int dx = get_global_id(0);
    int dy = get_global_id(1);

    // Check if the current thread corresponds to a valid pixel location in the destination image
    if (dx < dst_cols && dy < dst_rows)
    {
        // Compute the projected coordinates in the source image using the transformation matrix M
        float X0 = M[0] * dx + M[1] * dy + M[2];
        float Y0 = M[3] * dx + M[4] * dy + M[5];
        float W = M[6] * dx + M[7] * dy + M[8];
        // Correct the perspective division if W is not zero
        W = W != 0.0f ? INTER_TAB_SIZE / W : 0.0f;
        // Round the coordinates to nearest integers
        int X = rint(X0 * W), Y = rint(Y0 * W);

        // Convert the coordinates to short and apply bit shift for interpolation
        int sx = convert_short_sat(X >> INTER_BITS);
        int sy = convert_short_sat(Y >> INTER_BITS);

        // Clamp the source coordinates to ensure they are within the bounds of the source image
        short sx_clamp = clamp(sx, 0, src_cols - 1);
        short sx_p1_clamp = clamp(sx + 1, 0, src_cols - 1);
        short sy_clamp = clamp(sy, 0, src_rows - 1);
        short sy_p1_clamp = clamp(sy + 1, 0, src_rows - 1);
        // Fetch the pixel values at the computed source coordinates and their immediate neighbors
        int v0 = convert_int(src[mad24(sy_clamp, src_row_stride, src_offset + sx_clamp*src_px_stride)]);
        int v1 = convert_int(src[mad24(sy_clamp, src_row_stride, src_offset + sx_p1_clamp*src_px_stride)]);
        int v2 = convert_int(src[mad24(sy_p1_clamp, src_row_stride, src_offset + sx_clamp*src_px_stride)]);
        int v3 = convert_int(src[mad24(sy_p1_clamp, src_row_stride, src_offset + sx_p1_clamp*src_px_stride)]);

        // Calculate the sub-pixel precision parts of the source coordinates
        short ay = (short)(Y & (INTER_TAB_SIZE - 1));
        short ax = (short)(X & (INTER_TAB_SIZE - 1));
        float taby = 1.f/INTER_TAB_SIZE*ay;
        float tabx = 1.f/INTER_TAB_SIZE*ax;

        // Compute the index in the destination image where the pixel value will be stored
        int dst_index = mad24(dy, dst_row_stride, dst_offset + dx);

        // Compute the interpolation weights based on the sub-pixel positions
        int itab0 = convert_short_sat_rte( (1.0f-taby)*(1.0f-tabx) * INTER_REMAP_COEF_SCALE );
        int itab1 = convert_short_sat_rte( (1.0f-taby)*tabx * INTER_REMAP_COEF_SCALE );
        int itab2 = convert_short_sat_rte( taby*(1.0f-tabx) * INTER_REMAP_COEF_SCALE );
        int itab3 = convert_short_sat_rte( taby*tabx * INTER_REMAP_COEF_SCALE );

        // Apply the interpolation weights to the fetched pixel values to compute the final pixel value
        int val = v0 * itab0 +  v1 * itab1 + v2 * itab2 + v3 * itab3;

        // Convert the final value to uchar, applying a rounding correction for the remapping scale
        uchar pix = convert_uchar_sat((val + (1 << (INTER_REMAP_COEF_BITS-1))) >> INTER_REMAP_COEF_BITS);
        // Store the computed pixel value in the destination image
        dst[dst_index] = pix;
    }
}