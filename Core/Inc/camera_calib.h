/**
 * @file camera_calib.h
 * @brief GC2145 相机内参与标靶尺寸配置（QVGA 320x240）
 */

#ifndef CAMERA_CALIB_H
#define CAMERA_CALIB_H

/* GC2145 标定示例值（请替换为实际标定结果） */
#define CAMERA_FX       (280.0f)
#define CAMERA_FY       (280.0f)
#define CAMERA_CX       (160.0f)
#define CAMERA_CY       (120.0f)

/* 二维码边长（厘米） */
#define MARKER_SIZE_CM  (10.0f)

#endif /* CAMERA_CALIB_H */
