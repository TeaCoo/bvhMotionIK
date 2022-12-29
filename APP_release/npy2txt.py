import sys
import numpy as np
import os

npypath = str(sys.argv[1]) 
filename =os.path.splitext(npypath)[0].split("/")[-1]

npy=np.load(npypath)
# 转为矩阵
data = npy.reshape(npy.shape[0],-1)

# 保存为txt格式的文件
np.savetxt(r"./position/"+ filename +".txt",data,delimiter=',')