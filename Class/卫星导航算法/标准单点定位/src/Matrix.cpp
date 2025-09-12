#include"Matrix.h"
#include<iostream>
#include<cmath>
#include<vector>
#include<iomanip>

using namespace std;

/**
 * @brief 打印矩阵
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param matrix 矩阵数据（按行优先存储）
 * @param width 每个元素的宽度
 * @param precision 小数精度
 */
void PrintMatrix( int rows, int cols,const double* matrix, int width , int precision )
{
    if (rows <= 0 || cols <= 0) 
    {
        cerr << "错误：矩阵维度需为正整数 "<<endl;
        return;
    }
    if (!matrix) 
    {
        cerr << "错误：矩阵指针为空" <<endl;
        return;
    }
    cout << fixed << setprecision(precision);
    for (int i = 0; i < rows; ++i) {
        cout << (i == 0 ? "┌ " : (i == rows - 1 ? "└ " : "│ "));
        for (int j = 0; j < cols; ++j) {
            cout << setw(width)<< matrix[i * cols + j];
        }
        cout << (i == 0 ? " ┐" : (i == rows - 1 ? " ┘" : " │"))<< endl;
    }
}

/**
 * @brief 矩阵乘法 M3 = M1 * M2
 * @param m1 矩阵M1的行数
 * @param n1 矩阵M1的列数
 * @param m2 矩阵M2的行数
 * @param n2 矩阵M2的列数
 * @param M1 矩阵M1数据
 * @param M2 矩阵M2数据
 * @param M3 结果矩阵缓冲区
 */
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[])
{
    int i, j, k;
    double Sum;

    if ((n1 != m2) || (m1 <= 0) || (n1 <= 0) || (m2 <= 0) || (n2 <= 0))
    {
        printf("矩阵维数错误\n");
        exit(EXIT_FAILURE);
    }
    for (i = 0; i < m1; i++)
    {
        for (j = 0; j < n2; j++)
        {
            Sum = 0.0;

            for (k = 0; k < n1; k++)
            {
                Sum = Sum + *(M1 + i * n1 + k) * *(M2 + k * n2 + j);
            }

            *(M3 + i * n2 + j) = Sum;
        }
    }
}

/**
 * @brief 矩阵加法 M3 = M1 + M2
 * @param m1 矩阵M1的行数
 * @param n1 矩阵M1的列数
 * @param m2 矩阵M2的行数
 * @param n2 矩阵M2的列数
 * @param M1 矩阵M1数据
 * @param M2 矩阵M2数据
 * @param M3 结果矩阵缓冲区
 */
void MatrixAdd(int m1, int n1, const double M1[], const double M2[], double M3[])
{
    if (m1 <= 0 || n1 <= 0)
    {
        cerr << "错误：矩阵维度需为正整数" << endl;
        return;
    }
    if (!M1 || !M2 || !M3)
    {
        cerr << "错误：矩阵指针为空" << endl;
        return;
    }
    for (int i = 0; i < m1; ++i)
    {
        for (int j = 0; j < n1; ++j)
        {
            M3[i * n1 + j] = M1[i * n1 + j] + M2[i * n1 + j];
        }
    }
}

/**
 * @brief 矩阵转置
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param matrix 输入矩阵数据（按行优先存储）
 * @param transposed 输出转置矩阵缓冲区（需预分配cols×rows空间）
 */
void MatrixTranspose(int rows, int cols, const double* matrix, double* transposed)
{
    if (rows <= 0 || cols <= 0)
    {
        cerr << "错误：矩阵维度需为正整数" << endl;
        return;
    }
    if (!matrix || !transposed)
    {
        cerr << "错误：矩阵指针为空" << endl;
        return;
    }
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            transposed[j * rows + i] = matrix[i * cols + j];
        }
    }
}
/**
 * @brief 矩阵求逆函数（高斯-约旦消元法实现）
 * @param n       矩阵维度（n×n）
 * @param matrix  输入矩阵（按行存储的一维数组）
 * @param inverse 输出逆矩阵缓冲区（需预分配n×n空间）
 * @throws std::invalid_argument 当矩阵不可逆或输入非法时抛出异常
 * @return true 如果逆矩阵计算成功，false 如果输入参数无效或矩阵不可逆
 */
bool MatrixInverse(int n, const double* matrix, double* inverse) {
    // 参数有效性检查
    if (n <= 0) { 
        throw invalid_argument("矩阵维度必须为正整数");
        return false;
    }
    if (!matrix || !inverse)
    {
        throw invalid_argument("空指针传入");
        return false;
    }
    // 创建增广矩阵 [A|I]
    vector<double> aug(n * 2 * n);
    for (int i = 0; i < n; ++i) {
        // 复制原矩阵到左侧
        memcpy(&aug[i * 2 * n], &matrix[i * n], n * sizeof(double));
        // 右侧初始化为单位矩阵
        aug[i * 2 * n + n + i] = 1.0;
    }

    // 高斯-约旦消元主循环
    for (int col = 0; col < n; ++col) {
        // 寻找主元（列中最大绝对值）
        int pivot_row = col;
        double max_val = abs(aug[col * 2 * n + col]);
        for (int row = col + 1; row < n; ++row) {
            const double val = std::abs(aug[row * 2 * n + col]);
            if (val > max_val) {
                max_val = val;
                pivot_row = row;
            }
        }
        if (max_val < 1e-10) {
            throw invalid_argument("矩阵不可逆（奇异矩阵）");
            return false;
        }
        if (pivot_row != col) {
            for (int j = col; j < 2 * n; ++j) {
                swap(aug[col * 2 * n + j], aug[pivot_row * 2 * n + j]);
            }
        }
        const double pivot = aug[col * 2 * n + col];
        for (int j = col; j < 2 * n; ++j) {
            aug[col * 2 * n + j] /= pivot;
        }

        for (int row = 0; row < n; ++row) {
            if (row == col) continue;
            const double factor = aug[row * 2 * n + col];
            for (int j = col; j < 2 * n; ++j) {
                aug[row * 2 * n + j] -= factor * aug[col * 2 * n + j];
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        memcpy(&inverse[i * n], &aug[i * 2 * n + n], n * sizeof(double));
    }
    return true;
}

/**
 * @brief 矩阵求逆函数（高斯-约当消元法实现）
 * @param m       矩阵行列数
 * @param n       需要求逆的行列数
 * @param matrix  输入矩阵（按行存储的一维数组）
 * @param inverse 输出逆矩阵缓冲区（需预分配m×m空间）
 * @return true 如果逆矩阵计算成功，false 如果输入参数无效或矩阵不可逆
 */
bool MatrixInverse(int m, int n, double* matrix, double* inverse)
{
    // 初始化逆矩阵为全零
    std::fill(inverse, inverse + m * m, 0.0);
    // 处理边界情况：n 无效或超出范围
    if (n <= 0 || n > m) {
        return  false;
    }
    // 创建子矩阵副本
    double* subMatrix = new double[n * n];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            subMatrix[i * n + j] = matrix[i * m + j];
        }
    }
    // 创建增广矩阵 [子矩阵 | 单位矩阵]
    double* aug = new double[n * 2 * n];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + j] = subMatrix[i * n + j];
        }
        for (int j = 0; j < n; j++) {
            aug[i * 2 * n + n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    // 高斯-约当消去法（列主元）
    for (int k = 0; k < n; k++) {
        // 寻找列主元
        int maxRow = k;
        for (int i = k + 1; i < n; i++) {
            if (std::abs(aug[i * 2 * n + k]) > std::abs(aug[maxRow * 2 * n + k])) {
                maxRow = i;
            }
        }
        // 检查奇异矩阵
        if (std::abs(aug[maxRow * 2 * n + k]) < 1e-10) {
            delete[] subMatrix;
            delete[] aug;
            return false; // 保持逆矩阵为全零
        }
        // 行交换（如果需要）
        if (maxRow != k) {
            for (int j = 0; j < 2 * n; j++) {
                std::swap(aug[k * 2 * n + j], aug[maxRow * 2 * n + j]);
            }
        }
        // 主元归一化
        double pivot = aug[k * 2 * n + k];
        for (int j = 0; j < 2 * n; j++) {
            aug[k * 2 * n + j] /= pivot;
        }
        // 消元操作
        for (int i = 0; i < n; i++) {
            if (i == k) continue;
            double factor = aug[i * 2 * n + k];
            for (int j = 0; j < 2 * n; j++) {
                aug[i * 2 * n + j] -= factor * aug[k * 2 * n + j];
            }
        }
    }
    // 提取逆矩阵结果
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inverse[i * m + j] = aug[i * 2 * n + j + n];
        }
    }
    // 释放动态分配的内存
    delete[] subMatrix;
    delete[] aug;
}

/**
 * @brief 矩阵元素提取函数
 * @param rows 原矩阵行数
 * @param cols 原矩阵列数
 * @param a 被删除的行数组及其长度
 * @param a_len 被删除的行数组长度
 * @param b 被删除的列数组及其长度
 * @param b_len 被删除的列数组长度
 * @param m 原矩阵数据（按行优先存储）
 * @param n 输出结果矩阵数据（按行优先存储）
 */
void MatrixExtr(int rows, int cols, int a[], int a_len,  /* 被删除的行数组及其长度*/int b[], int b_len, double m[], double n[])
{
    int idx = 0;

    // 遍历原矩阵的每一行
    for (int i = 0; i < rows; i++) {
        // 检查当前行是否需要删除
        int is_row_removed = 0;
        for (int x = 0; x < a_len; x++) {
            if (i == a[x]) {
                is_row_removed = 1;
                break;
            }
        }
        if (is_row_removed) continue;

        // 遍历当前行的每一列
        for (int j = 0; j < cols; j++) {
            // 检查当前列是否需要删除
            int is_col_removed = 0;
            for (int y = 0; y < b_len; y++) {
                if (j == b[y]) {
                    is_col_removed = 1;
                    break;
                }
            }
            if (is_col_removed) continue;

            // 保留的元素按行优先顺序存入结果数组
            n[idx++] = m[i * cols + j];
        }
    }
}


/**
 * @brief 矩阵减法 M3 = M1 - M2
 * @param m1 矩阵M1的行数
 * @param n1 矩阵M1的列数
 * @param M1 矩阵M1数据
 * @param M2 矩阵M2数据
 * @param M3 结果矩阵缓冲区
 */
void MatrixSub(int m1, int n1, const double M1[], const double M2[], double M3[])
{
    int totalElements = m1 * n1;
    for (int i = 0; i < totalElements; i++)
    {
        M3[i] = M1[i] - M2[i];
    }
}
