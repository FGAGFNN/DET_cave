import cv2
import numpy as np
import onnxruntime

class Yolov8Seg():
    def __init__(self, modelpath,classes=80):
        self.net = onnxruntime.InferenceSession(modelpath, providers=['CUDAExecutionProvider'])
        self.confThreshold=0.5
        self.nmsThreshold=0.6
        self.maskThreshold=0.5
        self.inpWidth = 640
        self.inpHeight = self.inpWidth
        self.classes = classes
        self.segchannel = 32
        self.segwidth = 160
        self.segheight = 160
        self.color = []

        # for i in range(self.classes):
        #     self.color.append([randint(0, 256), randint(0, 256), randint(0, 256)])
        self.color = [ (255, 0, 255), (0, 0, 255), (255, 255, 255), (0, 255, 255)]
    def resize_image(self, srcimg, keep_ratio=True):
        top, left, newh, neww = 0, 0, self.inpWidth, self.inpHeight
        if keep_ratio and srcimg.shape[0] != srcimg.shape[1]:
            hw_scale = srcimg.shape[0] / srcimg.shape[1]
            if hw_scale > 1:
                newh, neww = self.inpHeight, int(self.inpWidth / hw_scale)
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                left = int((self.inpWidth - neww) * 0.5)
                img = cv2.copyMakeBorder(img, 0, 0, left, self.inpWidth - neww - left, cv2.BORDER_CONSTANT,
                                         value=(114, 114, 114))  # add border
            else:
                newh, neww = int(self.inpHeight * hw_scale), self.inpWidth
                img = cv2.resize(srcimg, (neww, newh), interpolation=cv2.INTER_AREA)
                top = int((self.inpHeight - newh) * 0.5)
                img = cv2.copyMakeBorder(img, top, self.inpHeight - newh - top, 0, 0, cv2.BORDER_CONSTANT,
                                         value=(114, 114, 114))
        else:
            img = cv2.resize(srcimg, (self.inpWidth, self.inpHeight), interpolation=cv2.INTER_AREA)
        return img, newh, neww, top, left

    def postprocessbox(self, frame, outs, padsize=None):
        
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        newh, neww, padh, padw = padsize
        ratioh, ratiow = frameHeight / newh, frameWidth / neww

        confidences = []
        boxes = []
        classIds = []
        temp_proto = []
        for detection in outs:
            scores = detection[4:4+self.classes]

            classId = np.argmax(scores)
            if classId>0:continue
            confidence = scores[classId]# * detection[4]
            if confidence > self.confThreshold:

                center_x = int((detection[0] - padw) * ratiow)
                center_y = int((detection[1] - padh) * ratioh)
                width = int(detection[2] * ratiow)#+10
                height = int(detection[3] * ratioh)#+10

                x = np.ceil(center_x - (width / 2))
                y = np.ceil(center_y - (height / 2))
                # width += 10
                # height += 10
                x = x if x>0 else 0
                y = y if y>0 else 0
                width = width if x+width<frameWidth else frameWidth-x
                height = height if y+height<frameHeight else frameHeight-y

                # 更新检测出来的框
                boxes.append([int(x), int(y), int(width), int(height)])
                confidences.append(float(confidence))
                classIds.append(classId)
                temp_proto.append(detection[4+self.classes:4+self.classes+self.segchannel])
        # Perform non maximum suppression to 00eliminate redundant overlapping boxes with
        # lower confidences.

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold,self.nmsThreshold)
        box=np.zeros((0,4))
        labels=np.zeros((0,))
        confs=np.zeros((0,))
        if len(idxs)>0:
            box_seq = idxs.flatten()
            box = np.array(boxes)[box_seq]
            labels = np.array(classIds)[box_seq]
            confs = np.array(confidences)[box_seq]
            temp_proto= np.array(temp_proto)[box_seq]
            box[:, 2] += box[:, 0]
            box[:, 3] += box[:, 1]
                
            return box.reshape(-1,4).astype('int32'),labels.astype('int32'),confs,temp_proto.reshape(-1,32)
        else:
            return np.array([]),np.array([]),np.array([]),np.array([])
    def postprocessmask(self,srcimg,maskouts,temp_proto,boxes,padsize):
        newh, neww, padh, padw = padsize
        protos=maskouts.reshape(self.segchannel,-1)
        matmulRes = temp_proto .dot( protos) #n*(160*160)
        
        masks = matmulRes.reshape(-1,self.segwidth,self.segheight)
        masks = 1 / (1+np.exp(-masks))
        left,top,width,height=int(padw/self.inpWidth*self.segwidth),int(padh/self.inpHeight*self.segheight),\
            int(self.segwidth-padw/2),int(self.segheight-padh/2)
        masks_roi=masks[:,top:top+height,left:left+width]
        output_mask=[]
        for i in range(masks.shape[0]):
            masks_roi_resize=cv2.resize(masks_roi[i],(srcimg.shape[1],srcimg.shape[0]))
            rect=boxes[i]
            mask=masks_roi_resize[rect[1]:rect[3],rect[0]:rect[2]] 
            mask[mask< self.maskThreshold]=0
            mask[mask>0]=1
            #缩小一点 减少误识别
            element = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5));
            mask=cv2.erode(mask, element)
            output_mask.append(mask)

        return output_mask
    def detect(self, srcimg):
        img, newh, neww, padh, padw = self.resize_image(srcimg)
        blob = cv2.dnn.blobFromImage(img, scalefactor=1 / 255.0, swapRB=True)

        out = self.net.run(['output0','output1'], {'images': blob})
        
        detouts,maskouts=out[0][0].T,out[1][0]
        
        # print(detouts)
        boxes,labels,confs,temp_proto = self.postprocessbox(srcimg, detouts, padsize=(newh, neww, padh, padw))
        if boxes.shape[0]==0:
            return np.zeros((0,4)),labels,confs,temp_proto
        output_mask=self.postprocessmask(srcimg, maskouts,temp_proto, boxes,padsize=(newh, neww, padh, padw))
        
        
        return boxes,labels,confs,output_mask
    
        
    def draw(self,srcimg,box,label,conf,output_mask):

        mask=np.zeros(srcimg.shape)
        for i in range(box.shape[0]):
            cv2.rectangle(srcimg, (box[i,0], box[i,1]), (box[i,2], box[i,3]), self.color[label[i]], 3)  # 画框
            cv2.putText(srcimg,'%d:%.2f'%(label[i],conf[i]), (box[i,0], box[i,1]), cv2.FONT_ITALIC, 1, self.color[label[i]], 3)
            for j in range(output_mask[i].shape[0]):
                for k in range(output_mask[i].shape[1]):
                    if output_mask[i][j,k]>0:
                        mask[box[i,1]+j,box[i,0]+k,:]=self.color[label[i]]
        mask=mask.astype('uint8')
        srcimg=cv2.addWeighted(srcimg, 1, mask, 0.3,1);
        return srcimg
        
