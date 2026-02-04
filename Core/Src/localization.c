/**
 * @file localization.c
 * @brief 定位与导航系统实现 - AprilTag位姿版本
 */

#include "localization.h"
#include "qr_comm.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

static RobotPose_t robot_pose = {0.0f, 0.0f, 0.0f, 0};
static NavigationTarget_t nav_target = {0};
static PIDParameters_t pid_params = {
    .kp_linear = 2.0f,   /* 增加比例系数 */
    .kd_linear = 0.2f,   /* 微分系数 */
    .kp_angular = 1.5f,  /* 增加角度比例系数 */
    .kd_angular = 0.1f,  /* 角度微分系数 */
};

static float last_linear_error = 0.0f;
static float last_angular_error = 0.0f;

/* AprilTag 世界坐标数据库 */
typedef struct {
    float world_x;    /* cm */
    float world_y;    /* cm */
    float heading;    /* degrees */
} TagWorldCoord_t;

static const TagWorldCoord_t tag_world_db[4] = {
    {0.0f, 0.0f, 0.0f},        /* TAG0 at origin */
    {40.0f, 0.0f, 0.0f},       /* TAG1 at (40,0) */
    {40.0f, 40.0f, 0.0f},      /* TAG2 at (40,40) */
    {0.0f, 40.0f, 0.0f},       /* TAG3 at (0,40) */
};

/* AprilTag 数量追踪（至少2个才能定位） */
#define MAX_TAG_IDS 10
#define MIN_TAGS_FOR_POSITIONING 2
static uint8_t detected_tag_ids[MAX_TAG_IDS];
static uint8_t detected_tag_count = 0;
static bool positioning_enabled = false;

/* 每轮缓存的标签位姿（只取每个标签的第1个值） */
typedef struct {
    bool valid;
    float x;
    float y;
    float theta;
} TagCache_t;

static TagCache_t tag_cache[4];
static uint8_t cached_tags = 0;

/* 多标签融合缓冲：存储最近的tag数据用于融合 */
typedef struct {
    int8_t id;
    float x;
    float y;
    float theta;
    uint32_t timestamp;
} TagHistory_t;

#define TAG_HISTORY_SIZE 4
static TagHistory_t tag_history[TAG_HISTORY_SIZE];
static uint8_t history_count = 0;

/* 简单卡尔曼滤波器 - 平滑位置跳变 */
typedef struct {
    float x, y, theta;           /* 当前估计位置 */
    float p_xx, p_yy, p_tt;      /* 协方差（简化，只用对角线） */
    float q;                      /* 过程噪声 */
    float r;                      /* 测量噪声 */
} KalmanFilter_t;

static KalmanFilter_t kf = {
    .x = 0, .y = 0, .theta = 0,
    .p_xx = 1.0f, .p_yy = 1.0f, .p_tt = 0.1f,
    .q = 0.01f,    /* 小过程噪声：信任预测 */
    .r = 0.5f,     /* 中等测量噪声：测量有误差 */
};

static void TagCache_Reset(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        tag_cache[i].valid = false;
        tag_cache[i].x = 0.0f;
        tag_cache[i].y = 0.0f;
        tag_cache[i].theta = 0.0f;
    }
    cached_tags = 0;
}

/* 位姿有效性筛选阈值（cm/deg） */
#define POSE_MAX_ABS_XY_CM  100.0f
#define POSE_MIN_ABS_Z_CM    10.0f
#define POSE_MAX_ABS_Z_CM   100.0f
#define POSE_MAX_ANGLE_DEG  360.0f

/**
 * @brief 验证位置数据有效性
 */
static bool Is_Pose_Valid(const QR_Data_t *qr)
{
    if (!qr || !qr->valid) return false;
    
    /* 支持两种mode：单tag(0-3)和融合位置(id=-1) */
    if (qr->id == -1) {
        /* 融合位置数据，更宽松的验证 */
        if (qr->x < -100 || qr->x > 100) return false;   /* 工作空间 ±100cm */
        if (qr->y < -100 || qr->y > 100) return false;
    } else if (qr->id >= 4) {
        /* 不支持其他 tag ID */
        return false;
    } else {
        /* 单标签模式验证 */
        /* 验证世界坐标范围 */
        if (qr->x < -100 || qr->x > 100) return false;
        if (qr->y < -100 || qr->y > 100) return false;
    }
    
    /* 验证偏航角范围 */
    if (qr->yaw > 360 || qr->yaw < -360) return false;

    return true;
}

/**
 * @brief 检查并记录检测到的 tag ID
 */
static void Track_Tag_ID(uint8_t tag_id)
{
    /* 融合位置数据，无需跟踪 */
    if ((int8_t)tag_id == -1) {
        return;
    }
    
    if (tag_id >= MAX_TAG_IDS) {
        printf("DEBUG: Invalid tag_id=%d (>=%d)\r\n", tag_id, MAX_TAG_IDS);
        return;
    }
    
    /* 检查是否已记录 */
    bool already_detected = false;
    for (uint8_t i = 0; i < detected_tag_count; i++) {
        if (detected_tag_ids[i] == tag_id) {
            already_detected = true;
            break;
        }
    }
    
    /* 添加新 tag */
    if (!already_detected && detected_tag_count < MAX_TAG_IDS) {
        detected_tag_ids[detected_tag_count++] = tag_id;
        printf("DEBUG: New TAG%d detected, total unique tags=%d\r\n", tag_id, detected_tag_count);
        
        /* 检查是否达到最小数量要求（至少2个tag） */
        if (detected_tag_count >= MIN_TAGS_FOR_POSITIONING && !positioning_enabled) {
            positioning_enabled = true;
            printf("DEBUG: *** POSITIONING ENABLED *** (tags=%d)\r\n", detected_tag_count);
        }
    }
}

/**
 * @brief 卡尔曼滤波平滑位置更新
 */
static void KalmanFilter_Update(float z_x, float z_y, float z_theta)
{
    /* 预测步骤（假设速度不变） */
    /* x = x + v_x*dt，这里简化为保持不变 */
    float p_xx = kf.p_xx + kf.q;
    float p_yy = kf.p_yy + kf.q;
    float p_tt = kf.p_tt + kf.q;
    
    /* 卡尔曼增益 */
    float k_x = p_xx / (p_xx + kf.r);
    float k_y = p_yy / (p_yy + kf.r);
    float k_t = p_tt / (p_tt + kf.r);
    
    /* 更新步骤 */
    kf.x = kf.x + k_x * (z_x - kf.x);
    kf.y = kf.y + k_y * (z_y - kf.y);
    kf.theta = kf.theta + k_t * (z_theta - kf.theta);
    
    /* 更新协方差 */
    kf.p_xx = (1.0f - k_x) * p_xx;
    kf.p_yy = (1.0f - k_y) * p_yy;
    kf.p_tt = (1.0f - k_t) * p_tt;
}

/**
 * @brief 用AprilTag位置数据更新机器人位置（支持单标签和多标签融合）
 */
void Update_Position_From_QR(const QR_Data_t *qr)
{
    if (!Is_Pose_Valid(qr)) {
        printf("DEBUG: Position INVALID for TAG%d (x=%d,y=%d,yaw=%d)\r\n", 
               (int8_t)qr->id, qr->x, qr->y, qr->yaw);
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    /* 记录 tag ID（融合数据会自动跳过） */
    Track_Tag_ID(qr->id);
    
    /* 启用定位 */
    if (!positioning_enabled) {
        positioning_enabled = true;
        if ((int8_t)qr->id == -1) {
            printf("DEBUG: *** POSITIONING ENABLED *** (multi-tag triangulation)\r\n");
        } else {
            printf("DEBUG: *** POSITIONING ENABLED *** (TAG%d detected)\r\n", qr->id);
        }
    }
    
    float z_x = (float)qr->x;
    float z_y = (float)qr->y;
    float z_theta = (qr->yaw * M_PI) / 180.0f;
    
    /* 多标签融合：收集单标签数据，如果有多个来自不同tag的数据，就融合 */
    if ((int8_t)qr->id >= 0) {
        /* 这是单标签数据，保存到历史缓冲 */
        if (history_count < TAG_HISTORY_SIZE) {
            tag_history[history_count].id = qr->id;
            tag_history[history_count].x = z_x;
            tag_history[history_count].y = z_y;
            tag_history[history_count].theta = z_theta;
            tag_history[history_count].timestamp = current_time;
            history_count++;
        } else {
            /* 缓冲满，覆盖最早的 */
            memmove(&tag_history[0], &tag_history[1], 
                   (TAG_HISTORY_SIZE - 1) * sizeof(TagHistory_t));
            tag_history[TAG_HISTORY_SIZE - 1].id = qr->id;
            tag_history[TAG_HISTORY_SIZE - 1].x = z_x;
            tag_history[TAG_HISTORY_SIZE - 1].y = z_y;
            tag_history[TAG_HISTORY_SIZE - 1].theta = z_theta;
            tag_history[TAG_HISTORY_SIZE - 1].timestamp = current_time;
        }
        
        /* 检查是否有足够新的多个不同tag的数据（时间差<500ms） */
        uint8_t unique_tags = 0;
        float fused_x = 0, fused_y = 0, fused_theta = 0;
        
        for (uint8_t i = 0; i < history_count; i++) {
            if (current_time - tag_history[i].timestamp < 500) {
                /* 检查是否已计数该tag */
                bool already_counted = false;
                for (uint8_t j = 0; j < i; j++) {
                    if (tag_history[j].id == tag_history[i].id) {
                        already_counted = true;
                        break;
                    }
                }
                if (!already_counted) {
                    fused_x += tag_history[i].x;
                    fused_y += tag_history[i].y;
                    fused_theta += tag_history[i].theta;
                    unique_tags++;
                }
            }
        }
        
        /* 如果有多个不同的tag，使用融合结果 */
        if (unique_tags >= 2) {
            z_x = fused_x / unique_tags;
            z_y = fused_y / unique_tags;
            z_theta = fused_theta / unique_tags;
            printf("[KALMAN] Multi-tag fusion: %d tags, pos=(%.1f,%.1f)\r\n", 
                   unique_tags, z_x, z_y);
        }
    }
    
    /* 使用卡尔曼滤波平滑位置 */
    KalmanFilter_Update(z_x, z_y, z_theta);
    
    /* 更新机器人位置 */
    robot_pose.x = kf.x;
    robot_pose.y = kf.y;
    robot_pose.theta = kf.theta;
    robot_pose.timestamp = current_time;
    
    if ((int8_t)qr->id == -1) {
        printf("DEBUG: [K210-FUSED] world=(%d,%d) yaw=%d → filtered=(%.1f,%.1f)\r\n", 
               (int)z_x, (int)z_y, qr->yaw,
               robot_pose.x, robot_pose.y);
    } else {
        printf("DEBUG: TAG%d world=(%d,%d) → filtered=(%.1f,%.1f)\r\n", 
               qr->id, (int)z_x, (int)z_y,
               robot_pose.x, robot_pose.y);
    }
}

/**
 * @brief 初始化定位系统
 */
void Localization_Init(void)
{
    robot_pose.x = 0.0f;
    robot_pose.y = 0.0f;
    robot_pose.theta = 0.0f;
    robot_pose.timestamp = HAL_GetTick();
    
    memset(detected_tag_ids, 0, sizeof(detected_tag_ids));
    detected_tag_count = 0;
    positioning_enabled = false;

    TagCache_Reset();
    
    nav_target.active = false;
    nav_target.tolerance = 5.0f;
}

/**
 * @brief 获取当前机器人位姿
 */
RobotPose_t* Get_Robot_Pose(void)
{
    return &robot_pose;
}

/**
 * @brief 获取当前位置
 */
Position_t Get_Current_Position(void)
{
    Position_t pos;
    pos.x = robot_pose.x;
    pos.y = robot_pose.y;
    pos.heading = robot_pose.theta * 180.0f / M_PI;
    return pos;
}

/**
 * @brief 检查是否已启用定位
 */
bool Is_Positioning_Enabled(void)
{
    return positioning_enabled;
}

/**
 * @brief 角度归一化到 [-π, π]
 */
static float Normalize_Angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief 设置导航目标
 */
void Set_Navigation_Target(float target_x, float target_y, float tolerance)
{
    nav_target.target_x = target_x;
    nav_target.target_y = target_y;
    nav_target.tolerance = tolerance;
    nav_target.active = true;
}

/**
 * @brief 导航更新
 */
bool Navigate_Update(void)
{
    if (!nav_target.active) {
        return true;
    }

    /* 计算目标距离和角度 */
    float dx = nav_target.target_x - robot_pose.x;
    float dy = nav_target.target_y - robot_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);
    float target_angle = atan2f(dy, dx);

    /* 检查是否到达目标 */
    if (distance < nav_target.tolerance) {
        nav_target.active = false;
        Motor_Stop_All();
        char nav_msg[80];
        snprintf(nav_msg, sizeof(nav_msg),
                "[NAV] Target reached (dist=%.1f < tol=%.1f)\r\n",
                distance, nav_target.tolerance);
        HAL_UART_Transmit(&huart3, (uint8_t *)nav_msg, strlen(nav_msg), 10);
        return true;
    }

    /* PID控制 */
    float linear_error = distance;
    float angular_error = Normalize_Angle(target_angle - robot_pose.theta);

    float linear_speed = pid_params.kp_linear * linear_error + 
                        pid_params.kd_linear * (linear_error - last_linear_error);
    float angular_speed = pid_params.kp_angular * angular_error + 
                         pid_params.kd_angular * (angular_error - last_angular_error);

    last_linear_error = linear_error;
    last_angular_error = angular_error;

    /* 限制速度并确保最小速度 */
    float max_speed = 400.0f;  /* 提高最大速度限制 */
    float min_speed = 30.0f;   /* 最小速度阈值，确保电机能动 */
    
    if (linear_speed > max_speed) linear_speed = max_speed;
    if (linear_speed < -max_speed) linear_speed = -max_speed;
    /* 如果速度太小但不为零，补到最小值 */
    if (linear_speed > 0 && linear_speed < min_speed) linear_speed = min_speed;
    if (linear_speed < 0 && linear_speed > -min_speed) linear_speed = -min_speed;

    if (angular_speed > max_speed) angular_speed = max_speed;
    if (angular_speed < -max_speed) angular_speed = -max_speed;
    if (angular_speed > 0 && angular_speed < min_speed/2) angular_speed = min_speed/2;
    if (angular_speed < 0 && angular_speed > -min_speed/2) angular_speed = -min_speed/2;

    /* 执行麦克纳姆轮控制 */
    Mecanum_Move(linear_speed, 0.0f, angular_speed);
    
    /* 调试输出：每200ms输出一次 */
    static uint32_t last_debug = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_debug >= 200) {
        last_debug = now;
        char nav_debug[150];
        snprintf(nav_debug, sizeof(nav_debug),
                "[NAV] d=%.1f a=%.0f° err_lin=%.1f err_ang=%.0f° v_lin=%.0f v_ang=%.0f\r\n",
                distance, target_angle * 180.0f / M_PI, 
                linear_error, angular_error * 180.0f / M_PI,
                linear_speed, angular_speed);
        HAL_UART_Transmit(&huart3, (uint8_t *)nav_debug, strlen(nav_debug), 10);
    }

    return false;
}

/**
 * @brief 获取设置的PID参数
 */
PIDParameters_t* Get_PID_Parameters(void)
{
    return &pid_params;
}

/**
 * @brief 设置PID参数
 */
void Set_PID_Parameters(const PIDParameters_t *params)
{
    if (params) {
        memcpy(&pid_params, params, sizeof(PIDParameters_t));
    }
}
