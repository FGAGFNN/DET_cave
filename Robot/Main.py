import multiprocessing
import cv2
import numpy as np
from Net import Yolov8Seg
from Realsense import D415
import time
from Com import Serial
import sys,os
from utils import Logger
from threading import Thread,Lock,Event
from multiprocessing import Process,Queue

# import schedule
modelpath = "e:/work/Mushroom/Project_luronggu/zhuabao_version1/model"
savepath = "e:/work/Mushroom/Project_luronggu/zhuabao_version1/result"
Transmit1 = Queue(1)
Recept1 = Queue(1)

lock=Lock()

def Serial_read(event,Recept1,serial):
    while not event.is_set():
        try:
            command = serial.com.read(100)
            if not len(command):
                continue
            print("Serial_read:",command)
            # AB BA 01 07 01 00 00 crc #01代表机械臂 07数据长度不含crc h01是采摘指令 00 00 是功能预留 CRC8
            if serial.CRC_Dector(command, len(command)) == 0:
                value,armx,army = command[2],0,0
                # print("value",value)
                if value==1:
                    if Recept1.empty():
                        Recept1.put([value,armx,army])
                if value ==3:
                    os.system("shutdown /s /t 1")
            else:
                print("CRC error")
        except Exception as e:
            print(f"Serial_read error: {e}")
            event.set()  # Signal to stop threads
            break
def Serial_Write(event,Transmit,serial):
    while not event.is_set():
        if not Transmit.empty() :
            lock.acquire()
            data = Transmit.get()
            # data = [data[0], 100, 200, 300, 150, 250, 350]
            if len(data)==1:
                # print("机械臂%d没有目标，发送数据"%data[0])
                serial.push_other_inform(data[0])
            else:
                print("机械臂%d有目标，发送坐标数据"%data[0])
                # out,arm =data[1:],data[0]
                out,arm =data[1:4],data[0]
                serial.push_right_axis2(out, arm)
            lock.release()
        else:
            continue
    print("退出串口写线程！！")

def Serial_Op(Recept1,Transmit1):
    stop_event = Event()

    while 1:
        try:
            stop_event.clear()
            serial = Serial('COM2',115200)
            print("串口进程启动成功")
            p=[]
            p.append(Thread(target=Serial_read,args=(stop_event,Recept1,serial)))
            p.append(Thread(target=Serial_Write,args=(stop_event,Transmit1,serial)))
            for i in range(len(p)):p[i].start()
            for i in range(len(p)):p[i].join()
        except Exception as e:
            print(f"串口异常: {e}")
            stop_event.set()
            time.sleep(1)
            print("尝试重新连接串口")
        finally:
            print("关闭串口")
            try:
                serial.com.close()
            except:
                pass



def consumer(ca_id,arm,Recept,Transmit,showpic):
    sys.stdout = Logger(savepath + '/'+time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())+'_arm%d.txt'%arm)
    # print(os.path.exists(r"E:/work/Mushroom/Project_luronggu/zhuabao_version1/data/000024.jpg"))
    # srcimg = cv2.imread(r"E:/work/Mushroom/Project_luronggu/zhuabao_version1/data/000024.jpg")
    width=640;height=480
    # print(srcimg.shape)
    # srcimg0 = cv2.resize(srcimg,(width,height))

    # try:
    d415 = D415(width=width,height=height);
    detectnet1 = Yolov8Seg('e:/work/Mushroom/Project_luronggu/zhuabao_version1/model/best.onnx',1)#
    if showpic:
        cv2.namedWindow('image%d'%arm, cv2.WINDOW_NORMAL)
        
    print("机械臂%d进程启动成功"%arm)
    # srcimg0 = None
    while 1:
        time.sleep(0.1)
        if not Recept.empty(): # 等待 Recept（来自 Serial_read）里有命令：  
            data = Recept.get()
            if len(data)==3:
                _,armx,army = data
            else:
                continue
        else:
            continue
        print("------------------------机械臂%d--------------------\n"%arm)
        print('机械臂%d,收到正确信号,开始检测%d,%d:'%(arm,armx,army),time.strftime(",%H_%M_%S", time.localtime()))
        # In[1] 检测
        srcimg = d415.get_d415_img()
        print(srcimg.shape)
    
        # srcimg=srcimg0.copy()
        # cv2.imwrite(time.strftime("data/%H_%M_%S.jpg", time.localtime()),srcimg)
        print('load image')
        # srcimg = srcimg0.copy()
        if showpic:
            canvas = np.zeros((height * 2, width * 2, 3), dtype=np.uint8)
            canvas[:height,:width]=srcimg#左上
            canvas[:height,width:]=d415.colorized_depth#右上
            cv2.imshow('image%d'%arm, canvas);
            key=cv2.waitKey(1)
        srcimgcopy1,srcimgcopy2,srcimgcopy3=srcimg.copy(),srcimg.copy(),srcimg.copy()
        # 得到每个果实的检测框与mask
        boxs,labels,confs,masks = detectnet1.detect(srcimg)
        # print('detect box:',box)
        if boxs.shape[0]<=0:
            print("机械臂%d,没有检测到蘑菇菌包,去下一个点位检测"%arm,time.strftime(",%H_%M_%S", time.localtime()))
            Transmit.put([arm])
            time.sleep(0.5);
            while not Recept.empty():Recept.get()
            continue
        print('机械臂%d,检测到%d个菌包'%(arm,boxs.shape[0]))
        
        if showpic: # 检测到目标的目标框
            srcimgcopy1 = detectnet1.draw(srcimgcopy1, boxs, labels, confs,masks)
            canvas[height:,:width]=srcimgcopy1#左下
            cv2.imshow('image%d'%arm, canvas);
            key=cv2.waitKey(1)
        
        #获取菌包mask对应的所有点云值求均值 作为抓取点的xyz
        box = np.hstack((boxs, labels.reshape(-1, 1)))
        boxes = d415.get_world_axis(box,masks)# boxes = [left,top,right,bottom,cx,cy,x,y,z,0,size]
        
        if boxes.shape[0]<=0:
            print('无目标测到有效三维坐标')
            continue
        print('共有%d目标测到有效三维坐标'%(boxes.shape[0]))
        if showpic:
            for i in range(boxes.shape[0]):
                print('%d:%d:%d'%(boxes[i,6]*1e3,boxes[i,7]*1e3,boxes[i,8]*1e3))
                cv2.rectangle(srcimgcopy3, (int(boxes[i,0]), int(boxes[i,1])), (int(boxes[i,2]), int(boxes[i,3])), (255,255,255), 3)  # 画框
                cv2.putText(srcimgcopy3,'%d:%d:%d'%(boxes[i,6]*1e3,boxes[i,7]*1e3,boxes[i,8]*1e3), (int(boxes[i,4]),int(boxes[i,5])), cv2.FONT_ITALIC, 1, (0,0,255), 3)
            canvas[height:,width:,:]=srcimgcopy3
            cv2.imshow('image%d'%arm, canvas);#右下
            key=cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                break
        # In[3] 排序
        # 从近到远排序
        boxes = boxes[boxes[:,8].argsort(),:]
        wait_for_push=[]
        for i in range(1):
            target = boxes[i,:]
            out = [int(target[6] * 1e3), int(target[7] * 1e3), int(target[8] * 1e3)]
            wait_for_push += out
        if len(wait_for_push)>0:
            wait_for_push = [arm]+wait_for_push
            Transmit.put(wait_for_push)
        else:
            Transmit.put([arm])
        while not Recept.empty():Recept.get()

    # In[]
    cv2.destroyAllWindows()
    # except Exception as e:
    #     print(f"相机异常: {e}")
    #     time.sleep(1)  # Wait before retrying
    #     print("尝试重新连接相机...")
    # finally:
    #     try:
    #         d415.stop()
    #     except:
    #         pass


if __name__ == '__main__':
    multiprocessing.freeze_support()

    showpic = 1
    caid =[]
    with open(modelpath +'/' + "caid.txt",'r') as f:
        ft = f.readlines()
        for line in ft:
            caid.append(line.strip())

    p = []
    p.append(Process(target=Serial_Op, args=( Recept1,Transmit1)))
    p.append(Process(target=consumer, args=( caid[0],1,Recept1,Transmit1,showpic)))
    
    for i in range(len(p)):
        p[i].daemon=True # 设置为守护进程
        p[i].start()
    for i in range(len(p)):
        p[i].join() # 等待所有子进程结束

