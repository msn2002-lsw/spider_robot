import time
import math
# import serial
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # 导入 3D 绘图模块
import matplotlib.pyplot as plt  # 修正导入 matplotlib.pyplot
# import serial_servo


class Robot_Control_Tool(object):

    def __init__(self, M_Hz) -> None:
        
        # 舵机ID设置
        self.RFJ1_ID = 1        # 右前腿ID
        self.RFJ2_ID = 2
        self.RFJ3_ID = 3

        self.RMJ1_ID = 4        # 右中腿ID
        self.RMJ2_ID = 5
        self.RMJ3_ID = 6

        self.RBJ1_ID = 7        # 右后腿ID
        self.RBJ2_ID = 8
        self.RBJ3_ID = 9

        self.LFJ1_ID = 10       # 左前腿ID
        self.LFJ2_ID = 11
        self.LFJ3_ID = 12

        self.LMJ1_ID = 13       # 左中腿ID
        self.LMJ2_ID = 14
        self.LMJ3_ID = 15

        self.LBJ1_ID = 16       # 左后腿ID
        self.LBJ2_ID = 17
        self.LBJ3_ID = 18

        self.Claw_FL_ID = 19        # 夹爪舵机ID
        self.Claw_FR_ID = 20
        self.Claw_BL_ID = 21
        self.Claw_BR_ID = 22

        # 舵机角度初始位置
        self.RFJ1_Angle_Init = 500        # 右前腿舵机初始角度
        self.RFJ2_Angle_Init = 500
        self.RFJ3_Angle_Init = 500
        self.RFJ4_Angle_Init = 500

        self.RMJ1_Angle_Init = 500      # 右中腿舵机初始角度
        self.RMJ2_Angle_Init = 500
        self.RMJ3_Angle_Init = 500

        self.RBJ1_Angle_Init = 500      # 右后腿舵机初始角度
        self.RBJ2_Angle_Init = 500
        self.RBJ3_Angle_Init = 500
        self.RBJ4_Angle_Init = 500

        self.LFJ1_Angle_Init = 500      # 左前腿舵机初始角度
        self.LFJ2_Angle_Init = 500
        self.LFJ3_Angle_Init = 500
        self.LFJ4_Angle_Init = 500

        self.LMJ1_Angle_Init = 500       # 左中腿舵机初始角度
        self.LMJ2_Angle_Init = 500
        self.LMJ3_Angle_Init = 500

        self.LBJ1_Angle_Init = 500       # 左后腿舵机初始角度
        self.LBJ2_Angle_Init = 500
        self.LBJ3_Angle_Init = 500
        self.LBJ4_Angle_Init = 500

        # 创建舵机控制工具
        # TTL = '/dev/ZZWUSB0'        #创建串口
        # Servo_control = serial_servo.SerialServo(TTL)      #创建舵机控制实例
        self.M_Hz = M_Hz

        # 机器人关节尺寸（mm）
        self.L1 = 40
        self.L2 = 49
        self.L3 = 132.5
        self.alpha = math.radians(40)      #单位/度，初始位置前后腿和中间腿的夹角

        # 机器人关节模型初始角度（°）
        self.joint_1 = 0
        self.joint_2 = 35
        self.joint_3 = -90

        # 机器人三角步态参数
        self.Tripod_Cyc_Time = 5        # 周期（s）
        self.Tripod_Cyc_Disp = 50  # 三角步态单周期运动距离(mm)
        self.Tripod_Cyc_Hig = 50  # 三角步态周期运动抬腿高度(mm)

        pass
    def  Angle_change_for_Motor(self,Joint_Group):
        
        # 角度赋予
        [
            RFJ1, RFJ2, RFJ3, RMJ1, RMJ2, RMJ3, RBJ1, RBJ2,
            RBJ3, LFJ1, LFJ2, LFJ3, LMJ1, LMJ2, LMJ3, LBJ1,
            LBJ2,LBJ3
        ] = Joint_Group

        # 右前
        RFJ1_A = ((RFJ1-self.joint_1)/240)*1000 + self.RFJ1_Angle_Init
        RFJ2_A = ((RFJ2-self.joint_2)/240)*1000 + self.RFJ2_Angle_Init
        RFJ3_A = ((RFJ3-self.joint_3)/240)*1000 + self.RFJ3_Angle_Init

        # 右中
        RMJ1_A = ((RMJ1-self.joint_1)/240)*1000 + self.RMJ1_Angle_Init
        RMJ2_A = ((RMJ2-self.joint_2)/240)*1000 + self.RMJ2_Angle_Init
        RMJ3_A = ((RMJ3-self.joint_3)/240)*1000 + self.RMJ3_Angle_Init

        # 右后
        RBJ1_A = ((RBJ1-self.joint_1)/240)*1000 + self.RBJ1_Angle_Init
        RBJ2_A = ((RBJ2-self.joint_2)/240)*1000 + self.RBJ2_Angle_Init
        RBJ3_A = ((RBJ3-self.joint_3)/240)*1000 + self.RBJ3_Angle_Init

        # 左前
        LFJ1_A = ((LFJ1-self.joint_1)/240)*1000 + self.LFJ1_Angle_Init
        LFJ2_A = ((LFJ2-self.joint_2)/240)*1000 + self.LFJ2_Angle_Init
        LFJ3_A = ((LFJ3-self.joint_3)/240)*1000 + self.LFJ3_Angle_Init

        # 左中
        LMJ1_A = ((LMJ1-self.joint_1)/240)*1000 + self.LMJ1_Angle_Init
        LMJ2_A = ((LMJ2-self.joint_2)/240)*1000 + self.LMJ2_Angle_Init
        LMJ3_A = ((LMJ3-self.joint_3)/240)*1000 + self.LMJ3_Angle_Init

        # 左后
        LBJ1_A = ((LBJ1-self.joint_1)/240)*1000 + self.LBJ1_Angle_Init
        LBJ2_A = ((LBJ2-self.joint_2)/240)*1000 + self.LBJ2_Angle_Init
        LBJ3_A = ((LBJ3-self.joint_3)/240)*1000 + self.LBJ3_Angle_Init

        Motor_A_Group = [[self.RFJ1_ID, RFJ1_A], [self.RFJ2_ID, RFJ2_A], [self.RFJ3_ID, RFJ3_A],
                         [self.RMJ1_ID, RMJ1_A], [self.RMJ2_ID, RMJ2_A], [self.RMJ3_ID, RMJ3_A],
                         [self.RBJ1_ID, RBJ1_A], [self.RBJ2_ID, RBJ2_A], [self.RBJ3_ID, RBJ3_A],
                         [self.LFJ1_ID, LFJ1_A], [self.LFJ2_ID, LFJ2_A], [self.LFJ3_ID, LFJ3_A],
                         [self.LMJ1_ID, LMJ1_A], [self.LMJ2_ID, LMJ2_A], [self.LMJ3_ID, LMJ3_A],
                         [self.LBJ1_ID, LBJ1_A], [self.LBJ2_ID, LBJ2_A], [self.LBJ3_ID, LBJ3_A]]
        
        return Motor_A_Group
    
    def Motor_Control(self, Motor_A_Group):
        pass

    # 机器人腿部正解
    def SDH_positive_solution(self, theta1, theta2, theta3):
        theta1=math.radians(theta1)     # 角度转换成弧度
        theta2=math.radians(theta2)
        theta3=math.radians(theta3)
        E=self.L1+self.L2*math.cos(theta2)+self.L3*math.cos(theta2+theta3)
        x=math.cos(theta1)*E
        y=math.sin(theta1)*E
        z=self.L2*math.sin(theta2)+self.L3*math.sin(theta2+theta3)
        return  [x,y,z]      # 返回足端坐标

    # 机器人腿部逆解
    def SDH_inverse_solution(self, x, y, z):
        C=math.pow(x,2)+math.pow(y,2)+math.pow(z,2)-2*self.L1*math.sqrt(math.pow(x,2)+math.pow(y,2))
        # print("C=",C)         输出测试
        B=C+math.pow(self.L1,2)+math.pow(self.L2,2)-math.pow(self.L3,2)
        # print("B=",B)
        A=math.sqrt(C+math.pow(self.L1,2))
        # print("A=",A)
        D=C+math.pow(self.L1,2)-math.pow(self.L2,2)-math.pow(self.L3,2)
        # print("D=",D)
        theta1=math.atan2(y,x)
        theta2=math.asin(z/A)+math.acos(B/(2*self.L2*A))
        theta3=-math.acos(D/(2*self.L2*self.L3))
        math.degrees(theta1)
        return [math.degrees(theta1),math.degrees(theta2),math.degrees(theta3)]      #返回三个角度值
    
    
    def Tripod_gait_plan(self, t):

        # 运动参数
        Swing_dis = self.Tripod_Cyc_Disp / 2
        T0 = self.Tripod_Cyc_Time / 2  # 三角步态分为俩个小周期，单位s
        V_R = self.Tripod_Cyc_Disp / self.Tripod_Cyc_Time  # 机器人运动速度，单位mm/s
        H_dis = self.Tripod_Cyc_Hig  # 机器人抬腿高度，单位mm
        High_f = 0.5

        # 腿初始位置
        [PX_init, PY_init, PZ_init] = self.SDH_positive_solution(self.joint_1, self.joint_2,self.joint_3) 

        # 运动规划,第一组（右前，左中，右后）
        Group_1_PX = PX_init
        if 0 <= t and t <= T0:      # 摆动相时足端规划
            Group_1_PY = PY_init + Swing_dis * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))
            if (0 <= t and t <= T0 * High_f):
                Group_1_PZ = PZ_init + H_dis * (t / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / (T0 * High_f)))
            elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
                Group_1_PZ = PZ_init + H_dis
            elif (T0 * (1 - High_f) < t and t <= T0):
                Group_1_PZ = PZ_init + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * (t - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))
        else:       # 支撑相时规划
            Group_1_PY = PY_init + Swing_dis - V_R * (t - T0)
            Group_1_PZ = PZ_init

        # 运动规划,第二组（左前，右中，左后）
        Group_2_PX = PX_init
        if 0 <= t and t <= T0:      # 支撑相时足端规划
            Group_2_PY = PY_init - V_R * t
            Group_2_PZ = PZ_init
        else:       # 摆动相时规划
            Group_2_PY = PY_init  - Swing_dis + Swing_dis * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))
            if (T0 <= t and t <= T0 * (1 + High_f)):
                Group_2_PZ = PZ_init + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / (T0 * High_f)))
            elif (T0 * (1 + High_f) < t and t <= T0 * (2 - High_f)):
                Group_2_PZ = PZ_init + H_dis
            else:
            # elif (T0 * (2 - High_f) < t and t <= 2 * T0):
                Group_2_PZ = PZ_init + H_dis - H_dis * (((t - T0) - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * ((t - T0) - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))
                
        print("Group_1_PX=", Group_1_PX, "Group_1_PY=", Group_1_PY, "Group_1_PZ=", Group_1_PZ)
        # print("Group_2_PX=", Group_2_PX, "Group_2_PY=", Group_2_PY, "Group_2_PZ=", Group_2_PZ)


        # 第一组腿关节解算
        [joint_1, joint_2, joint_3] = self.SDH_inverse_solution(Group_1_PX, Group_1_PY, Group_1_PZ)
        # 第二组腿关节解算
        [joint_4, joint_5, joint_6] = self.SDH_inverse_solution(Group_2_PX, Group_2_PY, Group_2_PZ)

        Joint_Group = [
            joint_1, joint_2, joint_3, 
            joint_4, joint_5, joint_6,
            joint_1, joint_2, joint_3, 
            joint_4, joint_5, joint_6,
            joint_1, joint_2, joint_3, 
            joint_4, joint_5, joint_6,
        ]
        return Joint_Group
    
    def Tripod_gait_control(self):

        Time_start = time.time()
        while True:

            Time_Current = time.time() - Time_start
        
            print("t=",Time_Current)
            A = self.Tripod_gait_plan(Time_Current)     # 获得对应关节角度

            B = self.Angle_change_for_Motor(A)      #将关节角度转换成舵机角度
            # print(B[0], B[1], B[3])
            self.Motor_Control(B)       #操作舵机

            # print(A[0], A[1], A[2])
            time.sleep(1 / spider.M_Hz)

            if Time_Current > spider.Tripod_Cyc_Time:
                break
        
        print("结束")

    def All_direction_Swing_Dis_Cal(self, direction, Swing_dis):

        # 运动参数
        theta = direction * math.pi / 180  # 转换成弧度
        Swing_dis = self.Tripod_Cyc_Disp / 2
        T0 = self.Tripod_Cyc_Time / 2  # 步态分为俩个小周期，单位s


        # 右前腿XY方向摆动距离
        RF_Swing_dis_X = Swing_dis * math.cos(self.alpha - theta)
        RF_Swing_dis_Y = - Swing_dis * math.sin(self.alpha - theta)
        RF_V_X = RF_Swing_dis_X / T0
        RF_V_Y = RF_Swing_dis_Y / T0

        # 右中腿XY方向摆动距离
        RM_Swing_dis_X = - Swing_dis * math.cos(theta)
        RM_Swing_dis_Y = - Swing_dis * math.sin(theta)
        RM_V_X = RM_Swing_dis_X / T0
        RM_V_Y = RM_Swing_dis_Y / T0

        # 右后腿XY方向摆动距离
        RB_Swing_dis_X = - Swing_dis * math.cos(math.pi - self.alpha - theta)
        RB_Swing_dis_Y = Swing_dis * math.sin(math.pi - self.alpha - theta)
        RB_V_X = RB_Swing_dis_X / T0
        RB_V_Y = RB_Swing_dis_Y / T0

        # 左前腿XY方向摆动距离
        LF_Swing_dis_X = - Swing_dis * math.cos(math.pi - self.alpha - theta)
        LF_Swing_dis_Y = - Swing_dis * math.sin(math.pi - self.alpha - theta)
        LF_V_X = LF_Swing_dis_X / T0
        LF_V_Y = LF_Swing_dis_Y / T0
        
        # 左中腿XY方向摆动距离
        LM_Swing_dis_X = - Swing_dis * math.cos(theta)
        LM_Swing_dis_Y = Swing_dis * math.sin(theta)
        LM_V_X = LM_Swing_dis_X / T0
        LM_V_Y = LM_Swing_dis_Y / T0

        # 左后腿XY方向摆动距离
        LB_Swing_dis_X = Swing_dis * math.cos(self.alpha - theta)
        LB_Swing_dis_Y = Swing_dis * math.sin(self.alpha - theta)
        LB_V_X = LB_Swing_dis_X / T0
        LB_V_Y = LB_Swing_dis_Y / T0

        # 摆动距离集合
        Swing_dis_group = [
            RF_Swing_dis_X, RF_Swing_dis_Y,
            RM_Swing_dis_X, RM_Swing_dis_Y,
            RB_Swing_dis_X, RB_Swing_dis_Y,
            LF_Swing_dis_X, LF_Swing_dis_Y,
            LM_Swing_dis_X, LM_Swing_dis_Y,
            LB_Swing_dis_X, LB_Swing_dis_Y
        ]
        # 摆动速度集合
        Swing_vel_group = [
            RF_V_X, RF_V_Y,
            RM_V_X, RM_V_Y,
            RB_V_X, RB_V_Y,
            LF_V_X, LF_V_Y,
            LM_V_X, LM_V_Y,
            LB_V_X, LB_V_Y
        ]
        print("RF_Swing_dis_X=", RF_Swing_dis_X, "RF_Swing_dis_Y=", RF_Swing_dis_Y)
        print("RM_Swing_dis_X=", RM_Swing_dis_X, "RM_Swing_dis_Y=", RM_Swing_dis_Y)
        print("RB_Swing_dis_X=", RB_Swing_dis_X, "RB_Swing_dis_Y=", RB_Swing_dis_Y)
        print("LF_Swing_dis_X=", LF_Swing_dis_X, "LF_Swing_dis_Y=", LF_Swing_dis_Y)
        print("LM_Swing_dis_X=", LM_Swing_dis_X, "LM_Swing_dis_Y=", LM_Swing_dis_Y)
        print("LB_Swing_dis_X=", LB_Swing_dis_X, "LB_Swing_dis_Y=", LB_Swing_dis_Y)
        return Swing_dis_group, Swing_vel_group

    def All_direction_plan(self, t, direction):

        # 运动参数
        theta = direction * math.pi / 180  # 转换成弧度
        Swing_dis = self.Tripod_Cyc_Disp / 2
        High_f = 0.5        #   滞空因子
        T0 = self.Tripod_Cyc_Time / 2  # 步态分为俩个小周期，单位s
        H_dis = self.Tripod_Cyc_Hig  # 机器人抬腿高度，单位mm

        # 腿初始位置，都相对于腿部坐标系
        [PX_init, PY_init, PZ_init] = self.SDH_positive_solution(self.joint_1, self.joint_2,self.joint_3)

        Swing_dis_Group, Swing_Vel_Group = self.All_direction_Swing_Dis_Cal(direction, Swing_dis)      # 计算摆动距离

        # 右前腿XY方向摆动距离
        RF_Swing_dis_X = Swing_dis_Group[0]
        RF_Swing_dis_Y = Swing_dis_Group[1]
        RF_V_X = Swing_Vel_Group[0]
        RF_V_Y = Swing_Vel_Group[1]

        # 右中腿XY方向摆动距离
        RM_Swing_dis_X = Swing_dis_Group[2]
        RM_Swing_dis_Y = Swing_dis_Group[3]
        RM_V_X = Swing_Vel_Group[2]
        RM_V_Y = Swing_Vel_Group[3]

        # 右后腿XY方向摆动距离
        RB_Swing_dis_X = Swing_dis_Group[4]
        RB_Swing_dis_Y = Swing_dis_Group[5]
        RB_V_X = Swing_Vel_Group[4]
        RB_V_Y = Swing_Vel_Group[5]

        # 左前腿XY方向摆动距离
        LF_Swing_dis_X = Swing_dis_Group[6]
        LF_Swing_dis_Y = Swing_dis_Group[7]
        LF_V_X = Swing_Vel_Group[6]
        LF_V_Y = Swing_Vel_Group[7]
        
        # 左中腿XY方向摆动距离
        LM_Swing_dis_X = Swing_dis_Group[8]
        LM_Swing_dis_Y = Swing_dis_Group[9]
        LM_V_X = Swing_Vel_Group[8]
        LM_V_Y = Swing_Vel_Group[9]

        # 左后腿XY方向摆动距离
        LB_Swing_dis_X = Swing_dis_Group[10]
        LB_Swing_dis_Y = Swing_dis_Group[11]
        LB_V_X = Swing_Vel_Group[10]
        LB_V_Y = Swing_Vel_Group[11]

        # 右前腿运动规划
        if 0 <= t and t <= T0:      # 摆动相时足端规划
            RF_PX = PX_init + RF_Swing_dis_X * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # x方向摆动规划

            RF_PY = PY_init + RF_Swing_dis_Y * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # y方向摆动规划

            if (0 <= t and t <= T0 * High_f):       # 抬腿高度规划
                RF_PZ = PZ_init + H_dis * (t / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / (T0 * High_f)))
            elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
                RF_PZ = PZ_init + H_dis
            elif (T0 * (1 - High_f) < t and t <= T0):
                RF_PZ = PZ_init + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * (t - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))
        else:       # 支撑相时规划
            RF_PX = PX_init + RF_Swing_dis_X - RF_V_X * (t - T0)
            RF_PY = PY_init + RF_Swing_dis_Y - RF_V_Y * (t - T0)
            RF_PZ = PZ_init

        [RF_Joint_1, RF_Joint_2, RF_Joint_3] = self.SDH_inverse_solution(RF_PX, RF_PY, RF_PZ)      # 右前腿关节解算

        # 右中腿运动规划
        if 0 <= t and t <= T0:      # 支撑相时足端规划
            RM_PX = PX_init + RM_Swing_dis_X - RM_V_X * (- t + T0)
            RM_PY = PY_init + RM_Swing_dis_Y - RM_V_Y * (- t + T0)
            RM_PZ = PZ_init
        else:       # 摆动相时规划
            RM_PX = PX_init + RM_Swing_dis_X - RM_Swing_dis_X * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))        # x方向摆动规划

            RM_PY = PY_init + RM_Swing_dis_Y - RM_Swing_dis_Y * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))        # y方向摆动规划

            if (T0 <= t and t <= T0 * (1 + High_f)):       # 抬腿高度规划
                RM_PZ = PZ_init + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / (T0 * High_f)))
            elif (T0 * (1 + High_f) < t and t <= T0 * (2 - High_f)):
                RM_PZ = PZ_init + H_dis
            else:
                RM_PZ = PZ_init + H_dis - H_dis * (((t - T0) - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * ((t - T0) - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))

        [RM_Joint_1, RM_Joint_2, RM_Joint_3] = self.SDH_inverse_solution(RM_PX, RM_PY, RM_PZ)      # 右中腿关节解算

        # 右后腿运动规划
        if 0 <= t and t <= T0:      # 摆动相时足端规划
            RB_PX = PX_init + RB_Swing_dis_X * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # x方向摆动规划

            RB_PY = PY_init + RB_Swing_dis_Y * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # y方向摆动规划

            if (0 <= t and t <= T0 * High_f):       # 抬腿高度规划
                RB_PZ = PZ_init + H_dis * (t / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / (T0 * High_f)))
            elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
                RB_PZ = PZ_init + H_dis
            elif (T0 * (1 - High_f) < t and t <= T0):
                RB_PZ = PZ_init + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * (t - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))
        else:       # 支撑相时规划
            RB_PX = PX_init + RB_Swing_dis_X - RB_V_X * (t - T0)
            RB_PY = PY_init + RB_Swing_dis_Y - RB_V_Y * (t - T0)
            RB_PZ = PZ_init

        [RB_Joint_1, RB_Joint_2, RB_Joint_3] = self.SDH_inverse_solution(RB_PX, RB_PY, RB_PZ)      # 右后腿关节解算

        # 左前腿运动规划
        if 0 <= t and t <= T0:      # 支撑相时足端规划
            LF_PX = PX_init + LF_V_X * t
            LF_PY = PY_init + LF_V_Y * t
            LF_PZ = PZ_init
        else:       # 摆动相时规划
            LF_PX = PX_init + LF_Swing_dis_X - LF_Swing_dis_X * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))        # x方向摆动规划

            LF_PY = PY_init + LF_Swing_dis_Y - LF_Swing_dis_Y * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))        # y方向摆动规划

            if (T0 <= t and t <= T0 * (1 + High_f)):       # 抬腿高度规划
                LF_PZ = PZ_init + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / (T0 * High_f)))
            elif (T0 * (1 + High_f) < t and t <= T0 * (2 - High_f)):
                LF_PZ = PZ_init + H_dis
            else:
                LF_PZ = PZ_init + H_dis - H_dis * (((t - T0) - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * ((t - T0) - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))

        [LF_Joint_1, LF_Joint_2, LF_Joint_3] = self.SDH_inverse_solution(LF_PX, LF_PY, LF_PZ)      # 左前腿关节解算

        # 左中腿运动规划
        if 0 <= t and t <= T0:      # 摆动相时足端规划
            LM_PX = PX_init + LM_Swing_dis_X * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # x方向摆动规划

            LM_PY = PY_init + LM_Swing_dis_Y * ((t / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / T0))        # y方向摆动规划

            if (0 <= t and t <= T0 * High_f):       # 抬腿高度规划
                LM_PZ = PZ_init + H_dis * (t / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * t / (T0 * High_f)))
            elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
                LM_PZ = PZ_init + H_dis
            elif (T0 * (1 - High_f) < t and t <= T0):
                LM_PZ = PZ_init + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * (t - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))
        else:       # 支撑相时规划
            LM_PX = PX_init + LM_Swing_dis_X - LM_V_X * (t - T0)
            LM_PY = PY_init + LM_Swing_dis_Y - LM_V_Y * (t - T0)
            LM_PZ = PZ_init

        [LM_Joint_1, LM_Joint_2, LM_Joint_3] = self.SDH_inverse_solution(LM_PX, LM_PY, LM_PZ)      # 左中腿关节解算

        # 左后腿运动规划
        if 0 <= t and t <= T0:      # 支撑相时足端规划
            LB_PX = PX_init + LB_V_X * t
            LB_PY = PY_init + LB_V_Y * t
            LB_PZ = PZ_init
        else:       # 摆动相时规划
            LB_PX = PX_init + LB_Swing_dis_X - LB_Swing_dis_X * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))       # x方向摆动规划
            LB_PY = PY_init + LB_Swing_dis_Y - LB_Swing_dis_Y * (((t - T0) / T0) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / T0))       # y方向摆动规划
            if (T0 <= t and t <= T0 * (1 + High_f)):       # 抬腿高度规划
                LB_PZ = PZ_init + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * np.pi) * math.sin(2 * np.pi * (t - T0) / (T0 * High_f)))
            elif (T0 * (1 + High_f) < t and t <= T0 * (2 - High_f)):
                LB_PZ = PZ_init + H_dis
            else:
                LB_PZ = PZ_init + H_dis - H_dis * (((t - T0) - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                     (2 * np.pi) * math.sin(2 * np.pi * ((t - T0) - T0 * (1 - High_f)) /
                                                                            (T0 * High_f)))

        [LB_Joint_1, LB_Joint_2, LB_Joint_3] = self.SDH_inverse_solution(LB_PX, LB_PY, LB_PZ)      # 左后腿关节解算

        # 足端位置集合
        foot_position = [
            RF_PX, RF_PY, RF_PZ,
            RM_PX, RM_PY, RM_PZ,
            RB_PX, RB_PY, RB_PZ,
            LF_PX, LF_PY, LF_PZ,
            LM_PX, LM_PY, LM_PZ,
            LB_PX, LB_PY, LB_PZ
        ]

        # 关节角度集合
        Joint_Angle = [
            RF_Joint_1, RF_Joint_2, RF_Joint_3,
            RM_Joint_1, RM_Joint_2, RM_Joint_3,
            RB_Joint_1, RB_Joint_2, RB_Joint_3,
            LF_Joint_1, LF_Joint_2, LF_Joint_3,
            LM_Joint_1, LM_Joint_2, LM_Joint_3,
            LB_Joint_1, LB_Joint_2, LB_Joint_3
        ]

        return foot_position, Joint_Angle
    
    def All_Direction_control(self, direction = None):
        # 运动参数
        if direction is None:
            direction = 45      # 运动方向，单位°

        Time_start = time.time()
        x = []
        y = []
        z = []
        while True:

            Time_Current = time.time() - Time_start

            print("------------------------------------")
            print("t=",Time_Current)
            [position, joint] = self.All_direction_plan(Time_Current, direction)     # 获得对应关节角度
            x.append(position[15])
            y.append(position[16])
            z.append(position[17])

            Motor_A_Group = self.Angle_change_for_Motor(joint)      #将关节角度转换成舵机角度
            self.Motor_Control(Motor_A_Group)       #操作舵机
            time.sleep(1 / self.M_Hz)

            # print(position[0], position[1], position[2])
            # print(position[3], position[4], position[5])
            # print(position[6], position[7], position[8])
            # print(position[9], position[10], position[11])
            # print(position[12], position[13], position[14])
            # print(position[15], position[16], position[17])

            # print(joint[0], joint[1], joint[2])
            # print(joint[3], joint[4], joint[5])
            # print(joint[6], joint[7], joint[8])
            # print(joint[9], joint[10], joint[11])
            # print(joint[12], joint[13], joint[14])
            # print(joint[15], joint[16], joint[17])

            if Time_Current > spider.Tripod_Cyc_Time:       # 运动周期结束
                break
        # self.plot_to_test(x, y, z)      # 绘制三维轨迹
        print("结束")

    def plot_to_test(self, x, y, z):

        num_points = len(x)
        colors = plt.cm.viridis(np.linspace(0, 1, num_points))  # 使用 Viridis 颜色映射，从浅到深

        # 绘制三维轨迹
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')  # 创建 3D 坐标轴
        for i in range(num_points):
            ax.scatter(x[i], y[i], z[i], color=colors[i], s=10)  # 单独绘制每个点，颜色渐变

        ax.set_xlabel("X 坐标")
        ax.set_ylabel("Y 坐标")
        ax.set_zlabel("Z 坐标")
        ax.set_title("机器人三维运动轨迹")
        plt.show()

if __name__ == "__main__":
    # 创建机器人控制实例
    spider = Robot_Control_Tool(5)

    spider.All_Direction_control(270)      # 运动方向，单位°
