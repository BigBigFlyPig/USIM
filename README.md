# USIM
双目校正+视差图获取（SGBM）

初次使用需要在 chessboard_image ，camera ，input_image , output_image 下新建文件夹 dir,dir,input_dir,output_dir
其中，前两个文件夹命名一致，为保存固定camera相关数据（投影校正矩阵，chessboard图片等）
将相机拍摄的测定板图片放入 chessboard_image/dir/ 下，编写image_list.txt
将需要计算视差图的图片放入 input_image/intput_dir/ 下，编写image_list

#include"camera.h"
int main()
{
  Camera camera(dir);
  camera.getDistparityMap(input_dir,output_dir);
  return 0;
}

opencv库需要重新配置
以上
