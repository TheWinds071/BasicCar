#ifndef LINE_FOLLOWER_INTERFACE_H
#define LINE_FOLLOWER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

    // 只暴露 C 语言能调用的函数原型
    void LineFollower_Init(void);
    void LineFollower_OnTimer(void);
    void LineFollower_SetSpeed(float speed);


#ifdef __cplusplus
}
#endif

void LineFollower_SetPID(uint8_t id, float kp, float ki, float kd);
void LineFollower_SetYaw();
void LineFollower_SetYawRef(float yaw_deg);

#endif // LINE_FOLLOWER_INTERFACE_H