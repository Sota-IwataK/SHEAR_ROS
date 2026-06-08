#include "kinematics.h"
#include "rclcpp/rclcpp.hpp" 
#include <cstring> // memcpy用
#include <cmath>   // 数学関数用


/**
 * @brief 行列の積の計算
 * 
 * @param _matrix1 入力行列１
 * @param _matrix2 入力行列２
 * @param _matrixOut 出力行列 (= 入力行列１ x 入力行列２)
 * @param _r1 入力行列1の行数
 * @param _c1 入力行列1の列数 (= 入力行列2の行数)
 * @param _c2 入力行列2の列数
 */
static void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,
    const int _r1, const int _c1, const int _c2)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _r1; i++)
    {
        for (j = 0; j < _c2; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _c1; k++)
                tmp += _matrix1[_c1 * i + k] * _matrix2[_c2 * k + j];

            _matrixOut[_c2 * i + j] = tmp;
        }
    }
}

/**
 * @brief 同次変換行列から回転行列を抽出
 * 
 * @param _transMat 同次変換行列（４ｘ４）入力
 * @param _rotationMat 回転行列（３ｘ３）出力
 */
static void GetRotationMat(const float* _transMat, float* _rotationMat)
{
    for (int i = 0; i < 9; i++)
    {
        if (i < 3)
            _rotationMat[i] = _transMat[i];
        else if (i >= 3 && i < 6)
            _rotationMat[i] = _transMat[i+1];
        else
            _rotationMat[i] = _transMat[i+2];
    }
}
//2025/04/29 tuika
static float clamp_angle_deg(float angle_deg, float min_deg, float max_deg)
{
    if (angle_deg < min_deg)
        return min_deg;
    if (angle_deg > max_deg)
        return max_deg;
    return angle_deg;
}


/**
 * @brief オイラー角を回転行列に変換（Z-Y-X系）
 * 
 * @param _eulerAngles 入力：オイラー角（X,Y,Zの順）
 * @param _rotationM 出力：回転行列（３ｘ３）
 */
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM) // rotz,roty,rotx
{
    float cos_rotZ, cos_rotY, cos_rotX, sin_rotZ, sin_rotY, sin_rotX;

    cos_rotX = cosf(_eulerAngles[0]);
    cos_rotY = cosf(_eulerAngles[1]);
    cos_rotZ = cosf(_eulerAngles[2]);
    sin_rotX = sinf(_eulerAngles[0]);
    sin_rotY = sinf(_eulerAngles[1]);
    sin_rotZ = sinf(_eulerAngles[2]);

    _rotationM[0] = cos_rotZ * cos_rotY;
    _rotationM[1] = cos_rotZ * sin_rotY * sin_rotX - sin_rotZ * cos_rotX;
    _rotationM[2] = cos_rotZ * sin_rotY * cos_rotX + sin_rotZ * sin_rotX;
    _rotationM[3] = sin_rotZ * cos_rotY;
    _rotationM[4] = sin_rotZ * sin_rotY * sin_rotX + cos_rotZ * cos_rotX;
    _rotationM[5] = sin_rotZ * sin_rotY * cos_rotX - cos_rotZ * sin_rotX;
    _rotationM[6] = -sin_rotY;
    _rotationM[7] = cos_rotY * sin_rotX;
    _rotationM[8] = cos_rotY * cos_rotX;
}

/**
 * @brief 回転行列をオイラー角に変換（Z-Y-X系）
 * 
 * @param _rotationM 入力：回転行列（３ｘ３）
 * @param _eulerAngles 出力：オイラー角（ロール、ピッチ、ヨーの順）
 */
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles)
{
    float rotZ, rotY, rotX, cb;

    if (fabs(_rotationM[6]) >= 1.0 - 0.0001) //cos(theta) = 0 // gimbal lock
    {
        if (_rotationM[6] < 0) // -sin(theta)
        {
            rotZ = 0.0f;
            rotY = (float)M_PI_2;
            rotX = atan2f(_rotationM[1], _rotationM[4]);
        }
        else
        {
            rotZ = 0.0f;
            rotY = -(float)M_PI_2;
            rotX = -atan2f(_rotationM[1], _rotationM[4]);
        }
    }
    else
    {
        rotY = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(rotY);
        rotZ = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        rotX = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = rotX;
    _eulerAngles[1] = rotY;
    _eulerAngles[2] = rotZ;
}

static void makeZero(float& x)
{
    if (fabs(x) < 0.001f)
        x = 0.0f;
}

/**
 * @brief Construct a new DOF5Kinematic::DOF5Kinematic object
 * 
 * @param L1 台座から第1軸、第2軸が直交する点の距離
 * @param L2 上腕の長さ
 * @param L3 前腕の長さ
 * @param L4 第4軸、第5軸が直交する点から手先の先端の距離
 */
DOF5Kinematic::DOF5Kinematic(float L1, float L2, float L3, float L4)
    : armConfig(ArmConfig_t{ L1, L2, L3, L4 })
{
    // DH parameters of AMIR 740
    float tmp_DH_matrix[5][4] = {
        // theta,             d,              a,                  alpha
        {0.0f,      armConfig.L_BASE,        0.0f,              (float)M_PI_2},
        {0.0f,              0.0f,         armConfig.L_ARM,      0.0f},
        {0.0f,              0.0f,         armConfig.L_FOREARM,  0.0f},
        {0.0f,              0.0f,            0.0f,             -(float)M_PI_2},
        {0.0f,      armConfig.L_GRIPPER,     0.0f,              0.0f}
    };
    memcpy(DH_matrix, tmp_DH_matrix, sizeof(tmp_DH_matrix));
}

/**
 * @brief 順運動学の計算
 * 
 * @param _inputJoint5D 入力：各関節角度 (mrad)
 * @param _outputPos5D 出力：手先の位置＋姿勢
 * @return true 
 */
bool DOF5Kinematic::solveFK(const DOF5Kinematic::Joint5D_t& _inputJoint5D, DOF5Kinematic::Pos5D_t& _outputPos5D)
{
    float theta_in[5];
    float theta[5], alpha[5];
    float cos_theta, sin_theta;
    float cos_alpha, sin_alpha;
    float A[5][16];
    float A02[16];
    float A03[16];
    float A04[16];
    float A05[16];
    float R_MAT[9];
    float euler_angles[3];

    for (int i = 0; i < 5; i++)
        theta_in[i] = _inputJoint5D.a[i] / 1000; // convert to rad

    for (int i = 0; i < 5; i++)
    {
        theta[i] = theta_in[i] + DH_matrix[i][0];
        cos_theta = cosf(theta[i]);
        sin_theta = sinf(theta[i]);
        alpha[i] = DH_matrix[i][3];
        cos_alpha = cosf(alpha[i]);
        sin_alpha = sinf(alpha[i]);

        //変換行列の項の計算
        A[i][0] = cos_theta;
        A[i][1] = -sin_theta * cos_alpha;
        A[i][2] = sin_theta * sin_alpha;
        A[i][3] = DH_matrix[i][2] * cos_theta;
        A[i][4] = sin_theta;
        A[i][5] = cos_theta * cos_alpha;
        A[i][6] = -cos_theta * sin_alpha;
        A[i][7] = DH_matrix[i][2] * sin_theta;
        A[i][8] = 0;
        A[i][9] = sin_alpha;
        A[i][10] = cos_alpha;
        A[i][11] = DH_matrix[i][1]; // d_i
        A[i][12] = 0;
        A[i][13] = 0;
        A[i][14] = 0;
        A[i][15] = 1;
    }

    MatMultiply(A[0], A[1], A02, 4, 4, 4);
    MatMultiply(A02, A[2], A03, 4, 4, 4);
    MatMultiply(A03, A[3], A04, 4, 4, 4);
    MatMultiply(A04, A[4], A05, 4, 4, 4); //A05 is the product of all 5 link transforms

    _outputPos5D.X = A05[3]; // m
    _outputPos5D.Y = A05[7];
    _outputPos5D.Z = A05[11];

    // get Rotation Matrix(orientation) and Position Matrix from A05
    GetRotationMat(A05, R_MAT);

    RotMatToEulerAngle(R_MAT, euler_angles);

    _outputPos5D.RotX = euler_angles[0]; // rad
    _outputPos5D.RotY = euler_angles[1];
    _outputPos5D.RotZ = euler_angles[2];

    return true;
}

/**
 * @brief 逆運動学の計算、台座の座標系を基準とし、手先の位置を姿勢を入力して、各関節の角度を求める
 * 
 * @param _inputPos5D 入力：目標位置(X,Y,Zの順、単位：10^-4 m);目標姿勢（ロール、ピッチ、ヨーの順、単位：10^-4 rad）
 * @param _outputSolves 出力：各関節の角度（4種類の解、単位：rad）
 * @return true 解が求められる場合
 * @return false 解が求められない場合
 */
bool DOF5Kinematic::solveIK(const DOF5Kinematic::Pos5D_t& _inputPos5D, DOF5Kinematic::IKSolves_t& _outputSolves)
{
    float P05[6];
    float ROT_MAT[9];

    P05[0] = _inputPos5D.X ; // convert into m
    P05[1] = _inputPos5D.Y ;
    P05[2] = _inputPos5D.Z ;
    P05[3] = _inputPos5D.RotX / 10000 ; // convert into rad
    P05[4] = _inputPos5D.RotY / 10000 ;
    P05[5] = _inputPos5D.RotZ / 10000 ;

    EulerAngleToRotMat(&(P05[3]), ROT_MAT);

    //theta1 (2種類の解)
    float t1[2];
    //check if px, py is 0
    if (sqrtf(P05[0] * P05[0] + P05[1] * P05[1]) <= 0.000001)
    {
        // Serial.println("joint1: ERROR \n");
        RCLCPP_WARN(rclcpp::get_logger("kinematics"), "joint1: ERROR (px, py too small)");
        return false;
    }
    else
    {
        t1[0] = atan2f(P05[1], P05[0]);
        t1[1] = atan2f(-P05[1], -P05[0]);
        makeZero(t1[0]);
        makeZero(t1[1]);
    }

    //theta5
    float t5a[2], t5b[2], t5[2];
    for (int i = 0; i < 2; i++)
    {
        t5a[i] = -ROT_MAT[0] * sinf(t1[i]) + ROT_MAT[3] * cosf(t1[i]);
        t5b[i] = -ROT_MAT[1] * sinf(t1[i]) + ROT_MAT[4] * cosf(t1[i]);
        t5[i] = atan2f(t5a[i], t5b[i]);
        makeZero(t5[i]);
    }

    //theta234 = theta2 + theta3 + theta4
    float t234a[2], t234b[2], t234[2];
    for (int i = 0; i < 2; i++)
    {
        t234a[i] = -ROT_MAT[2] * cosf(t1[i]) - ROT_MAT[5] * sinf(t1[i]);
        t234b[i] = ROT_MAT[8];
        t234[i] = atan2f(t234a[i], t234b[i]);
        makeZero(t234[i]);
    }

    //幾何解法
    //theta3
    float px, py, pz;
    float px2, py2, pz2;
    float l2, l3;
    float l22, l32;
    float c3;
    float t3;
    // ジョインんと２からジョイント４の位置
    px = -armConfig.L_GRIPPER * ROT_MAT[2] + P05[0];
    py = -armConfig.L_GRIPPER * ROT_MAT[5] + P05[1];
    pz = -armConfig.L_GRIPPER * ROT_MAT[8] + P05[2] - armConfig.L_BASE; // ベースからジョイント２のオフセット

    makeZero(px);
    makeZero(py);
    makeZero(pz);

    px2 = px * px;
    py2 = py * py;
    pz2 = pz * pz;

    l2 = armConfig.L_ARM;
    l3 = armConfig.L_FOREARM;
    l22 = l2 * l2;
    l32 = l3 * l3;

    c3 = (px2 + py2 + pz2 - l22 - l32) / (2 * l2 * l3);

    if (fabs(c3 - 1.0f) <= 0.001f)
        t3 = 0.00f;
    else
        t3 = -atan2f(sqrtf(1 - c3 * c3), c3);

    //theta2
    float t2A, t2B, t2[2];
    t2A = atan2f(pz, sqrtf(px2 + py2));
    t2B = atan2f(l3 * sinf(fabs(t3)), l2 + l3 * cosf(fabs(t3)));

    // 場合分け処理（他の象限にある時）
    for (int i = 0; i < 2; i++)
    {
        //theta1 in 1st quadrant
        if (0 <= t1[i] && t1[i] < (float)M_PI_2)
        {
            // eef in 1st and 4th quadrant
            if (px >= 0.0 && py >= 0.0)
                t2[i] = t2A + t2B;
            else
                t2[i] = (float)M_PI - t2A + t2B;
        }
        //theta1 in 2nd quadrant
        else if ((float)M_PI_2 <= t1[i] && t1[i] < (float)M_PI)
        {
            // eef in 1st and 4th quadrant
            if (px <= 0.0 && py >= 0.0)
                t2[i] = t2A + t2B;
            else
                t2[i] = (float)M_PI - t2A + t2B;
        }
        //theta1 in 3rd quadrant
        else if (-(float)M_PI <= t1[i] && t1[i] < -(float)M_PI_2)
        {
            // eef in 1st and 4th quadrant
            if (px <= 0.0 && py <= 0.0)
                t2[i] = t2A + t2B;
            else
                t2[i] = (float)M_PI - t2A + t2B;
        }
        //theta1 in 4th quadrant
        else if (-(float)M_PI_2 <= t1[i] && t1[i] < 0)
        {
            // eef in 1st and 4th quadrant
            if (px >= 0.0 && py <= 0.0)
                t2[i] = t2A + t2B;
            else
                t2[i] = (float)M_PI - t2A + t2B;
        }
    }

    //theta4
    float t4[2];
    t4[0] = t234[0] - t2[0] - t3;
    t4[1] = t234[1] - t2[1] - t3;

    float solvesConfig[4][5] = {
        {t1[0], t2[0], t3, t4[0], t5[0]},
        {t1[1], t2[1], t3, t4[1], t5[1]},
        {t1[0], t2[0], t3, t4[0] - 2 * (float)M_PI, t5[0]},
        {t1[1], t2[1], t3, t4[1] - 2 * (float)M_PI, t5[1]}
    };

    // for (int i = 0; i < 4; i++) {
    //     // --- クランプ範囲（単位はdeg） ---
    //     const float joint1_min = -5934.0f;
    //     const float joint1_max =  0.0f;
    //     const float joint2_min = -2356.0f;
    //     const float joint2_max =  0.0f;
    //     const float joint3_min = 0.0f;
    //     const float joint3_max =  2793.0f;
    //     const float joint4_min = -3403.0f;
    //     const float joint4_max = 0.0f;
    //     const float joint5_min = 0.0f;
    //     const float joint5_max =  5515.0f;
    
    //     solvesConfig[i][0] = clamp_angle_deg(solvesConfig[i][0] * 1000.0f, joint1_min, joint1_max) / 1000.0f;
    //     solvesConfig[i][1] = clamp_angle_deg(solvesConfig[i][1] * 1000.0f, joint2_min, joint2_max) / 1000.0f;
    //     solvesConfig[i][2] = clamp_angle_deg(solvesConfig[i][2] * 1000.0f, joint3_min, joint3_max) / 1000.0f;
    //     solvesConfig[i][3] = clamp_angle_deg(solvesConfig[i][3] * 1000.0f, joint4_min, joint4_max) / 1000.0f;
    //     solvesConfig[i][4] = clamp_angle_deg(solvesConfig[i][4] * 1000.0f, joint5_min, joint5_max) / 1000.0f;
    // }
    
    // クランプ後にコピーする
    for (int i = 0; i < 4; i++)
        memcpy(_outputSolves.config[i].a, solvesConfig[i], sizeof(_outputSolves.config[i].a));
    


    // for (int i = 0; i < 4; i++)
    //     memcpy(_outputSolves.config[i].a, solvesConfig[i], sizeof(_outputSolves.config[i].a));

    return true;
}

DOF5Kinematic::Joint5D_t
operator-(const DOF5Kinematic::Joint5D_t& _joints1, const DOF5Kinematic::Joint5D_t& _joints2)
{
    DOF5Kinematic::Joint5D_t tmp{};
    for (int i = 0; i < 5; i++) tmp.a[i] = _joints1.a[i] - _joints2.a[i];
    return tmp;
}

DOF5Kinematic::Joint5D_t
operator+(const DOF5Kinematic::Joint5D_t& _joints1, const DOF5Kinematic::Joint5D_t& _joints2)
{
    DOF5Kinematic::Joint5D_t tmp{};
    for (int i = 0; i < 5; i++) tmp.a[i] = _joints1.a[i] + _joints2.a[i];
    return tmp;
}