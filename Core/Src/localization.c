/**
 * @file localization.c
 * @brief 定位与导航系统实现（基于二维码角点的平面PnP和加权融合）
 */

#include "localization.h"
#include "qr_comm.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float Normalize_Angle(float angle);
static void Blend_Pose(float meas_x, float meas_y, float meas_theta, uint32_t now, float quality);

static RobotPose_t robot_pose = {0.0f, 0.0f, 0.0f, 0};
static NavigationTarget_t nav_target = {0};
static bool positioning_enabled = false;
static PIDParameters_t pid_params = {
    .kp_linear = 0.5f,
    .kd_linear = 0.1f,
    .kp_angular = 0.3f,
    .kd_angular = 0.05f,
};

static float last_linear_error = 0.0f;
static float last_angular_error = 0.0f;
static float last_body_x_error = 0.0f;
static float last_body_y_error = 0.0f;
static uint32_t last_nav_tick = 0;
/* 相机内参与标靶尺寸（单位：像素、厘米） */
static float cam_fx = 280.0f;  /* 请按标定结果更新 */
static float cam_fy = 280.0f;
static float cam_cx = 160.0f;  /* QVGA 默认中心 */
static float cam_cy = 120.0f;
static float marker_size_cm = 5.0f; /* 二维码边长（cm），请按实际更新 */

void Set_Camera_Intrinsics(float fx, float fy, float cx, float cy)
{
    cam_fx = fx; cam_fy = fy; cam_cx = cx; cam_cy = cy;
}

void Set_Marker_Size(float size_cm)
{
    marker_size_cm = size_cm;
}

/* 求 8x8 线性方程组的解（高斯消元，简单实现） */
static int solve8(const float A[8][8], const float b[8], float x[8])
{
    float M[8][9];
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) M[i][j] = A[i][j];
        M[i][8] = b[i];
    }
    for (int col = 0; col < 8; col++) {
        int piv = col;
        float best = fabsf(M[piv][col]);
        for (int r = col+1; r < 8; r++) {
            float v = fabsf(M[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-6f) return -1;
        if (piv != col) {
            for (int k = col; k <= 8; k++) {
                float tmp = M[piv][k]; M[piv][k] = M[col][k]; M[col][k] = tmp;
            }
        }
        float div = M[col][col];
        for (int k = col; k <= 8; k++) M[col][k] /= div;
        for (int r = 0; r < 8; r++) if (r != col) {
            float factor = M[r][col];
            for (int k = col; k <= 8; k++) M[r][k] -= factor * M[col][k];
        }
    }
    for (int i = 0; i < 8; i++) x[i] = M[i][8];
    return 0;
}

/* 单码平面PnP：由角点和内参求相机在QR所在平面坐标系下的位置与朝向 */
static int SolvePoseFromCorners(const QR_Data_t *qr, float *out_x, float *out_y, float *out_theta)
{
    if (!qr->corners_valid) return -4;

    float s = marker_size_cm;
    float X[4] = { -s*0.5f,  s*0.5f,  s*0.5f, -s*0.5f };
    float Y[4] = { -s*0.5f, -s*0.5f,  s*0.5f,  s*0.5f };

    float u[4], v[4];
    for (int i = 0; i < 4; i++) {
        float ui = (float)qr->corner_x[i];
        float vi = (float)qr->corner_y[i];
        u[i] = (ui - cam_cx) / cam_fx;
        v[i] = (vi - cam_cy) / cam_fy;
    }

    float A[8][8] = {0};
    float bvec[8] = {0};
    for (int i = 0; i < 4; i++) {
        int r = i*2;
        A[r][0] = X[i]; A[r][1] = Y[i]; A[r][2] = 1.0f;
        A[r][6] = -u[i]*X[i]; A[r][7] = -u[i]*Y[i];
        bvec[r] = u[i];

        A[r+1][3] = X[i]; A[r+1][4] = Y[i]; A[r+1][5] = 1.0f;
        A[r+1][6] = -v[i]*X[i]; A[r+1][7] = -v[i]*Y[i];
        bvec[r+1] = v[i];
    }
    float xsol[8];
    if (solve8(A, bvec, xsol) != 0) return -1;

    float h11=xsol[0], h12=xsol[1], h13=xsol[2];
    float h21=xsol[3], h22=xsol[4], h23=xsol[5];
    float h31=xsol[6], h32=xsol[7];
    float h1[3] = { h11, h21, h31 };
    float h2[3] = { h12, h22, h32 };
    float h3[3] = { h13, h23, 1.0f };

    float n1 = sqrtf(h1[0]*h1[0]+h1[1]*h1[1]+h1[2]*h1[2]);
    float n2 = sqrtf(h2[0]*h2[0]+h2[1]*h2[1]+h2[2]*h2[2]);
    float lam = (n1 + n2) * 0.5f;
    if (lam < 1e-6f) return -2;

    float r1[3] = { h1[0]/lam, h1[1]/lam, h1[2]/lam };
    float r2[3] = { h2[0]/lam, h2[1]/lam, h2[2]/lam };
    float dot12 = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];
    r2[0] -= dot12 * r1[0]; r2[1] -= dot12 * r1[1]; r2[2] -= dot12 * r1[2];
    float n2o = sqrtf(r2[0]*r2[0]+r2[1]*r2[1]+r2[2]*r2[2]);
    if (n2o < 1e-6f) return -3;
    r2[0]/=n2o; r2[1]/=n2o; r2[2]/=n2o;
    float r3[3] = { r1[1]*r2[2]-r1[2]*r2[1], r1[2]*r2[0]-r1[0]*r2[2], r1[0]*r2[1]-r1[1]*r2[0] };

    float t[3] = { h3[0]/lam, h3[1]/lam, h3[2]/lam };

    float Cx = -(r1[0]*t[0] + r2[0]*t[1] + r3[0]*t[2]);
    float Cy = -(r1[1]*t[0] + r2[1]*t[1] + r3[1]*t[2]);
    float Cz = -(r1[2]*t[0] + r2[2]*t[1] + r3[2]*t[2]);

    *out_x = Cx; *out_y = Cy; float yaw = atan2f(r1[1], r1[0]); *out_theta = yaw;
    (void)Cz;
    return 0;
}

/* 使用PnP直接更新世界位姿（假设QR坐标系与世界坐标系平行） */
bool Update_Pose_From_QR_PnP(const QR_Data_t *qr)
{
    if (!qr) return false;
    float px, py, pth;
    if (SolvePoseFromCorners(qr, &px, &py, &pth) != 0) return false;
    uint32_t now = HAL_GetTick();
    Blend_Pose(qr->world_x + px, qr->world_y + py, pth, now, 0.65f);
    return true;
}

/* 近期二维码观测缓冲，用于多码融合 */
typedef struct { float wx; float wy; float theta; float weight; uint32_t ts; } QRObs_t;
#define QR_OBS_MAX   8
#define QR_OBS_WINDOW_MS  120U
static QRObs_t qr_obs_buf[QR_OBS_MAX];
static uint8_t qr_obs_count = 0;
static float hypotf2(float dx, float dy) { return sqrtf(dx*dx + dy*dy); }
static void qr_obs_prune(uint32_t now){ uint8_t w=0; for(uint8_t i=0;i<qr_obs_count;i++){ if((now-qr_obs_buf[i].ts)<=QR_OBS_WINDOW_MS){ qr_obs_buf[w++]=qr_obs_buf[i];}} qr_obs_count=w; }
static void qr_obs_add(const QRObs_t *o){ if(qr_obs_count<QR_OBS_MAX) qr_obs_buf[qr_obs_count++]=*o; else { for(uint8_t i=1;i<QR_OBS_MAX;i++) qr_obs_buf[i-1]=qr_obs_buf[i]; qr_obs_buf[QR_OBS_MAX-1]=*o; }}

#define POSE_BLEND_MIN_ALPHA       0.18f
#define POSE_BLEND_MAX_ALPHA       0.72f
#define POSE_JUMP_REJECT_CM        45.0f
#define POSE_MAX_SPEED_CM_S        260.0f
#define NAV_MAX_LINEAR_PWM         520.0f
#define NAV_MAX_STRAFE_PWM         420.0f
#define NAV_MAX_ANGULAR_PWM        260.0f
#define NAV_MIN_PWM                45.0f

static float Clamp_Float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float Apply_Min_Output(float value, float min_abs)
{
    if (value > 0.0f && value < min_abs) return min_abs;
    if (value < 0.0f && value > -min_abs) return -min_abs;
    return value;
}

static void Blend_Pose(float meas_x, float meas_y, float meas_theta, uint32_t now, float quality)
{
    if (robot_pose.timestamp == 0U) {
        robot_pose.x = meas_x;
        robot_pose.y = meas_y;
        robot_pose.theta = Normalize_Angle(meas_theta);
        robot_pose.timestamp = now;
        positioning_enabled = true;
        return;
    }

    uint32_t dt_ms = now - robot_pose.timestamp;
    float dx = meas_x - robot_pose.x;
    float dy = meas_y - robot_pose.y;
    float jump = hypotf2(dx, dy);

    if (dt_ms < 1000U && jump > POSE_JUMP_REJECT_CM) {
        float speed_cm_s = jump * 1000.0f / (float)(dt_ms > 0U ? dt_ms : 1U);
        if (speed_cm_s > POSE_MAX_SPEED_CM_S) {
            return;
        }
    }

    float alpha = Clamp_Float(quality, POSE_BLEND_MIN_ALPHA, POSE_BLEND_MAX_ALPHA);
    if (dt_ms > 500U) {
        alpha = POSE_BLEND_MAX_ALPHA;
    }

    robot_pose.x += alpha * dx;
    robot_pose.y += alpha * dy;
    robot_pose.theta = Normalize_Angle(robot_pose.theta + alpha * Normalize_Angle(meas_theta - robot_pose.theta));
    robot_pose.timestamp = now;
    positioning_enabled = true;
}

void Localization_Init(void)
{
    robot_pose.x = 0.0f; robot_pose.y = 0.0f; robot_pose.theta = 0.0f; robot_pose.timestamp = HAL_GetTick();
    positioning_enabled = false;
    nav_target.active = false; nav_target.tolerance = 5.0f; qr_obs_count = 0;
    last_linear_error = 0.0f; last_angular_error = 0.0f; last_body_x_error = 0.0f; last_body_y_error = 0.0f; last_nav_tick = 0U;
}

static float Normalize_Angle(float angle){ while(angle>M_PI) angle-=2.0f*M_PI; while(angle<-M_PI) angle+=2.0f*M_PI; return angle; }

void Update_Position_From_QR(const QR_Data_t *qr)
{
    if (qr == NULL) return;
    uint32_t now = HAL_GetTick();

    if (qr->pose_valid) {
        float theta = Normalize_Angle(qr->heading_deg * (float)M_PI / 180.0f);
        Blend_Pose(qr->world_x, qr->world_y, theta, now, 0.55f);
        return;
    }

    if (!qr->corners_valid) return;

    float dx = (float)qr->corner_x[1] - (float)qr->corner_x[0];
    float dy = (float)qr->corner_y[1] - (float)qr->corner_y[0];
    float edge_len = hypotf2(dx, dy); if (edge_len < 1.0f) edge_len = 1.0f;
    float theta_est = atan2f(dy, dx);
    QRObs_t obs = { .wx = qr->world_x, .wy = qr->world_y, .theta = theta_est, .weight = edge_len, .ts = now };
    qr_obs_prune(obs.ts); qr_obs_add(&obs);
    float sum_w=0, fx=0, fy=0, csum=0, ssum=0;
    for(uint8_t i=0;i<qr_obs_count;i++){ const QRObs_t *q=&qr_obs_buf[i]; sum_w+=q->weight; fx+=q->weight*q->wx; fy+=q->weight*q->wy; csum+=q->weight*cosf(q->theta); ssum+=q->weight*sinf(q->theta); }
    if (sum_w>0.0f){ Blend_Pose(fx/sum_w, fy/sum_w, atan2f(ssum, csum), now, 0.35f); }
    else { Blend_Pose(qr->world_x, qr->world_y, theta_est, now, 0.25f); }
}

RobotPose_t* Get_Robot_Pose(void){ return &robot_pose; }

bool Is_Positioning_Enabled(void){ return positioning_enabled; }

void Set_Navigation_Target(float x, float y, float tolerance){ nav_target.target_x=x; nav_target.target_y=y; nav_target.tolerance=tolerance; nav_target.active=true; last_linear_error=0.0f; last_angular_error=0.0f; last_body_x_error=0.0f; last_body_y_error=0.0f; last_nav_tick=HAL_GetTick(); }

void Stop_Navigation(void){ nav_target.active=false; Motor_Stop_All(); }

bool Check_Target_Reached(void){ if(!nav_target.active) return false; float dx=nav_target.target_x-robot_pose.x; float dy=nav_target.target_y-robot_pose.y; return sqrtf(dx*dx+dy*dy) < nav_target.tolerance; }

void Calculate_Navigation_Error(float *distance, float *angle_error){ float dx=nav_target.target_x-robot_pose.x; float dy=nav_target.target_y-robot_pose.y; *distance=sqrtf(dx*dx+dy*dy); float target_theta=atan2f(dy,dx); *angle_error=Normalize_Angle(target_theta-robot_pose.theta); }

bool Navigate_Update(void)
{
    if (!nav_target.active) return false;
    uint32_t now = HAL_GetTick();
    float dt = (last_nav_tick == 0U) ? 0.02f : (float)(now - last_nav_tick) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;
    last_nav_tick = now;

    float distance, angle_error; Calculate_Navigation_Error(&distance, &angle_error);
    extern UART_HandleTypeDef huart3;
    char nav_debug[256]; snprintf(nav_debug, sizeof(nav_debug), "NAV: pos(%.1f,%.1f) target(%.1f,%.1f) dist=%.1f angle=%.2f\r\n", robot_pose.x, robot_pose.y, nav_target.target_x, nav_target.target_y, distance, angle_error); HAL_UART_Transmit(&huart3, (uint8_t *)nav_debug, strlen(nav_debug), 10);
    if (distance < nav_target.tolerance) { Motor_Stop_All(); nav_target.active=false; return true; }

    float dx = nav_target.target_x - robot_pose.x;
    float dy = nav_target.target_y - robot_pose.y;
    float ct = cosf(robot_pose.theta);
    float st = sinf(robot_pose.theta);
    float body_x_error = ct * dx + st * dy;
    float body_y_error = -st * dx + ct * dy;

    float vx = pid_params.kp_linear * body_x_error + pid_params.kd_linear * (body_x_error - last_body_x_error) / dt;
    float vy = pid_params.kp_linear * body_y_error + pid_params.kd_linear * (body_y_error - last_body_y_error) / dt;
    float omega = pid_params.kp_angular * angle_error + pid_params.kd_angular * (angle_error - last_angular_error) / dt;

    float slow_scale = Clamp_Float(distance / 80.0f, 0.35f, 1.0f);
    vx = Clamp_Float(vx, -NAV_MAX_LINEAR_PWM * slow_scale, NAV_MAX_LINEAR_PWM * slow_scale);
    vy = Clamp_Float(vy, -NAV_MAX_STRAFE_PWM * slow_scale, NAV_MAX_STRAFE_PWM * slow_scale);
    omega = Clamp_Float(omega, -NAV_MAX_ANGULAR_PWM, NAV_MAX_ANGULAR_PWM);

    if (fabsf(body_x_error) > nav_target.tolerance) vx = Apply_Min_Output(vx, NAV_MIN_PWM);
    if (fabsf(body_y_error) > nav_target.tolerance) vy = Apply_Min_Output(vy, NAV_MIN_PWM);

    snprintf(nav_debug, sizeof(nav_debug), "CTRL: vx=%.1f vy=%.1f omega=%.1f\r\n", vx, vy, omega); HAL_UART_Transmit(&huart3, (uint8_t *)nav_debug, strlen(nav_debug), 10);
    last_linear_error = distance; last_angular_error = angle_error; last_body_x_error = body_x_error; last_body_y_error = body_y_error;
    Mecanum_Move(vx, vy, omega);
    return false;
}



Position_t Get_Current_Position(void){ Position_t pos; pos.x = robot_pose.x; pos.y = robot_pose.y; pos.heading = robot_pose.theta * 180.0f / M_PI; return pos; }

/* PID helper for alternative navigation (kept for backward compatibility) */
PIDParameters_t* Get_PID_Parameters(void){ return &pid_params; }

void Set_PID_Parameters(const PIDParameters_t *params){ if (params) memcpy(&pid_params, params, sizeof(PIDParameters_t)); }
