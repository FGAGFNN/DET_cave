import numpy as np
import cv2
import sys


def keep_max_contours(mask,rect=False):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
    if len(contours)>=1:
        cv_contours = []
        threshold=0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= threshold:
                threshold=area
                max_contours=contour
        cv_contours.append(max_contours)
        mask = np.zeros_like(mask)
        cv2.fillPoly(mask, cv_contours, 1)
        if rect:
            
            rbox=cv2.minAreaRect(max_contours) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            # print(rbox)
            #以x轴正方向，顺时针旋转 平行的第一条边为w，得到的旋转角度
            box = cv2.boxPoints(rbox) # 获取最小外接矩形的4个顶点坐标
            #x最小的点 沿顺时针
            box = np.int0(box)
            
            cv2.drawContours(mask, [box], 0, (1, 0, 0), 1)
            return mask,box
    else:
        if rect:
            return mask,[]
        
    return mask

        

class Logger(object):
	def __init__(self, filename="Default.log"):
		self.terminal = sys.stdout
		self.filename = filename[:-4]+'_test.txt'
		# self.log = open(filename, "a")
		# 可以选择"w"
		#self.log = open(filename, "a", encoding="utf-8")  # 防止编码错误
	
	def write(self, message):
		self.terminal.write(message)
		#self.log.write(message)
		with open(self.filename, "a", encoding="gbk") as f:
			f.write(message)
	
	def flush(self):
		pass
	
	def reset(self):
		#self.log.close()
		sys.stdout = self.terminal

