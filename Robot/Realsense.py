# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import numpy as np
from scipy import stats
import cv2
import time
    
class D415():
    def __init__(self, id="211122063188",width=640,height=480,file=None):
        self.pipeline = rs.pipeline()
        config = rs.config()
        self.file=file
        if file is not None:
            rs.config.enable_device_from_file(config, file, repeat_playback=True)
            config.enable_stream(rs.stream.depth, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
        else:
            config.enable_device(id) if id is not None else 1#036422061147 037522063102
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        # self.depth_to_disparity = rs.disparity_transform(True)
        # self.disparity_to_depth = rs.disparity_transform(False)
        # self.decimation = rs.decimation_filter()
        # self.spatial = rs.spatial_filter()
        # self.temporal = rs.temporal_filter()
        # self.hole_filling = rs.hole_filling_filter()
        # self.hole_filling.set_option(rs.option.holes_fill,0)
        # self.spatial.set_option(rs.option.holes_fill,2)
        self.colorizer = rs.colorizer()

        self.profile=self.pipeline.start(config)
        # sensor_color = self.profile.get_device().first_color_sensor()
        # sensor_color.set_option(rs.option.enable_auto_exposure,True)
        #sensor_color.set_option(rs.option.exposure,10)


        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.depth_offset = depth_sensor.get_option(rs.option.depth_units);
        print(self.depth_scale,self.depth_offset)

        if file is not None:
            device = self.pipeline.get_active_profile().get_device()
            playback = device.as_playback()
            playback.set_real_time(False)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
    def stop(self):
        cv2.destroyAllWindows()
        self.pipeline.stop()
    def get_d415_img(self):
        # 获取颜色和深度的框架集
        frames = self.pipeline.wait_for_frames()
        # 将深度框与颜色框对齐
        aligned_frames = self.align.process(frames)
        # 获取对齐的帧
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        
        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        # self.aligned_depth_frame = self.decimation.process(self.aligned_depth_frame)
        
        self.intrin_part = [self.depth_intrin.ppx, 
                        self.depth_intrin.ppy, 
                        self.depth_intrin.fx, 
                        self.depth_intrin.fy]
        #print(intrin_part)
        
        # aligned_depth_frame = self.depth_to_disparity.process(self.aligned_depth_frame)
        # aligned_depth_frame = self.spatial.process(aligned_depth_frame)
        # aligned_depth_frame = self.temporal.process(aligned_depth_frame)
        # aligned_depth_frame = self.disparity_to_depth.process(aligned_depth_frame)
        # aligned_depth_frame = self.hole_filling.process(aligned_depth_frame)
        # self.aligned_depth_frame=aligned_depth_frame
        
        color_frame = aligned_frames.get_color_frame()
        #self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # print(10*'-')
        #depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        rgb_img = np.asanyarray(color_frame.get_data())
        if self.file is not None:rgb_img=rgb_img[:,:,::-1]
        self.depth_img = np.asanyarray(self.aligned_depth_frame.get_data())
        self.colorized_depth = np.asanyarray(self.colorizer.colorize(self.aligned_depth_frame).get_data())

        return rgb_img 

    def Get_Dis2(self, point):  # 计算整个框的所有点的三维坐标
        min_x, min_y, width, height = [int(i) for i in point]
        matz = self.depth_img[min_y:min_y + height, min_x:min_x + width] *self.depth_scale+self.depth_offset
        x = np.arange(min_x, min_x + width)
        y = np.arange(min_y, min_y + height)
        X, Y = np.meshgrid(x, y, indexing='ij')
        X_flat = X.flatten()
        Y_flat = Y.flatten()
        matx2 = (X_flat - self.intrin_part[0]) * matz.flatten() / self.intrin_part[2]
        maty2 = (Y_flat - self.intrin_part[1]) * matz.flatten() / self.intrin_part[3]
        dis_mask = np.stack([matx2.reshape(height, width), maty2.reshape(height, width), matz], axis=2)
        return dis_mask

    def get_distance(self,box, mask):#只测量果实分割出来的像素点的位置信息
        '''计算整个框的所有的距离,然后乘上mask,得到mask包含的距离值'''
        t_1 = time.time()
        mask_dis = self.Get_Dis2([box[0],box[1],box[2]-box[0],box[3]-box[1]])
        mask_dis = mask_dis * mask[:,:,np.newaxis]
        t_2 = time.time()
        print("计算框内点距离用时:%.3f"%(t_2-t_1))
        #计算果实的坐标----以所有能测到的点的平均距离来决定
        array = mask_dis.reshape(-1,3)
        array = np.delete(array, np.s_[array[:, -1] == 0], axis=0)
        
        try:
            temp_ = array
            # print(temp_)
            # 剔除距离为0，与距离太远的点 ，且距离大于均值+3倍方差的离群点 
            mean,std=np.mean(temp_[:, -1]),np.std(temp_[:, -1])
            # up=mean + 1 * std
            # down=mean - 1 * std
            conf_intveral = stats.norm.interval(0.5, loc=mean, scale=std) # 50%概率
            up=conf_intveral[1]
            down=conf_intveral[0]
            temp_ = np.delete(temp_, np.s_[temp_[:, -1] > up], axis=0)
            temp_ = np.delete(temp_, np.s_[temp_[:, -1] < down], axis=0)
            t_3= time.time()
            print("m+3std用时:%.3f"%(t_3-t_2))
        except Exception as e:
            print('meanstd error:',e)
            temp_ = array
        try:
            if temp_.shape[0] > 0:
                x,y,z=np.mean(temp_, axis=0)
                return x,y,z
            else:
                return 0, 0, 0
        except Exception as e:
            print('result error:',e)
            return 0, 0, 0
        
    def getdeltaxy(self,cx,cy,z0):
        x, y, z = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [cx, cy], z0)
        x1, y1, z1 = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [cx+1, cy+1], z0)
        return abs(x1-x),abs(y1-y)
    
    
    def get_world_axis(self,boxes,mask_list,value=0,direct_out=False):
        temp_ = np.zeros((len(boxes), 11))
        for i in range(boxes.shape[0]):
            box=boxes[i,:]
            mask_i=mask_list[i].astype(np.uint8)
            try: 
                #质心
                index=np.where(mask_i>0)
                cx=int(np.mean(index[1]))+int(box[0])
                cy=int(np.mean(index[0]))+int(box[1])
            except Exception as e:#
                print("该box没有对应的mask")
                continue
            t1=time.time()
            x0, y0, z0 = self.get_distance(box,mask_i)
            t2=time.time()
            print("计算距离用时:%.3f"%(t2-t1))
            deltax,deltay = self.getdeltaxy(cx, cy, z0)
            sizex,sizey = deltax*(box[2]-box[0]),deltay*(box[3]-box[1])
            size=int(min(sizex,sizey)*100)
            #print('size:',size)

            temp_[i, :] = np.array([boxes[i,0],boxes[i,1],boxes[i,2],boxes[i,3],
                                    cx,cy,
                                    x0, y0, z0,
                                    boxes[i,4],size])
            
        if direct_out:
            return temp_
        # print(temp_)
        num1=temp_.shape[0]
        condition2=np.where(temp_[:, 8] >0.8)[0].tolist()
        condition3=np.where(temp_[:, 8] <0.2)[0].tolist()
        num2=len(list(set(condition2 + condition3)))
        a=list(set(condition2 + condition3))
        temp_ = np.delete(temp_, a, axis=0)
        print("机械臂%d检测到%d个目标,距离超限的有%d个,剩余%d个有效目标"%(value,num1,num2,temp_.shape[0]))
        
        return temp_