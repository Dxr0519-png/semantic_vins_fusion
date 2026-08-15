# 二值实例掩码（0/1，行主序，尺寸可相对原图下采样以省带宽）
int32 width              # 掩码宽（像素）
int32 height             # 掩码高（像素）
uint8[] data             # 长度 = width*height，取值 0 或 1
