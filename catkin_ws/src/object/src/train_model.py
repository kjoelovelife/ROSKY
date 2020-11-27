#!/usr/bin/env python3

import os, sys, argparse, errno, yaml, time
import cv2, numpy 
import rospy, rospkg, threading
import torch, torchvision
import torch.optim as optim
import torch.nn.functional as function
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms


class Train_Model_Node(object):
 
    ####
    # local param :
    # ros param   : 
    # service     : 
    ####  

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing......".format(self.node_name))

        # set/get ros param
        self.param_model = rospy.get_param("~model","alexnet")
        self.trained_model_name = rospy.get_param("~train_model_name","trained_" + self.param_model)

        # set local param
        self.yaml_dict = {}
        self.kind_of_classifier = 0

        # read label
        _read = self.read_param_from_file() # will get self.yaml_dict, self.kind_of_classifier
        
        # initial model
        _data = self.rule_for_datasets()
        _AI   = self.neural_network(model=self.param_model,param_pretrained=True)
        _cuda = self.use_cuda(cuda=True) 


    def rule_for_datasets(self):
        folder = rospkg.RosPack().get_path('object') + "/image" 
        self.dataset = datasets.ImageFolder(
                           folder,
                           transforms.Compose([
                               transforms.ColorJitter(0.1, 0.1, 0.1, 0.1),
                               transforms.Resize((224, 224)),
                               transforms.ToTensor(),             
                               transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                                           ]) 
                        )
        # split dataset
        self.train_dataset, self.test_dataset = torch.utils.data.random_split(self.dataset, [len(self.dataset)-50, 50])

        # create data loaders to load data in batches
        self.train_loader = torch.utils.data.DataLoader(self.train_dataset, batch_size=16, shuffle=True, num_workers=4)
        self.test_loader = torch.utils.data.DataLoader(self.test_dataset, batch_size=16, shuffle=True, num_workers=4)

    def neural_network(self,model: str="alexnet", param_pretrained: bool=False):
        # reference : https://pytorch.org/docs/stable/torchvision/models.html
        model_list = [
                      "resnet18", "alexnet", "squeezenet", "vgg16", 
                      "densenet", "inception", "googlenet", "shufflenet", 
                      "mobilenet", "resnet34", "wide_resnet50_2", "mnasnet" 
                     ]

        if model in model_list:
            rospy.loginfo("You use model [{}]. Need some time to load model...".format(model))
            start_time = rospy.get_time()
            if model == "resnet18":
                self.model = models.resnet18(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512,self.kind_of_classifier)
            elif model == "alexnet":
                self.model = models.alexnet(pretrained=param_pretrained)
                self.model.classifier[6] = torch.nn.Linear(self.model.classifier[6].in_features, self.kind_of_classifier)
            elif model == "squeezenet":
                self.model = models.squeezenet1_1(pretrained=param_pretrained)
                self.model.classifier[1] = torch.nn.Conv2d(self.model.classifier[1].in_features, self.kind_of_classifier, kernel_size=1)
                self.num_classes = self.kind_of_classifier
            elif model == "vgg16":
                self.model = models.vgg16(pretrained=param_pretrained)
            elif model == "densenet":
                self.model = models.densenet161(pretrained=param_pretrained)
            elif model == "inception":
                self.model = models.inception_v3(pretrained=param_pretrained)
            elif model == "googlenet":
                self.model = models.googlenet(pretrained=param_pretrained)
            elif model == "shufflenet":
                self.model = models.shufflenet_v2_x1_0(pretrained=param_pretrained)
            elif model == "mobilenet":
                self.model = models.mobilenet_v2(pretrained=param_pretrained)
            elif model == "resnext50_32x4d":
                self.model = models.resnext50_32x4d(pretrained=param_pretrained)
            elif model == "resnet34": 
                self.model = models.resnet34(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512,self.kind_of_classifier)
            elif model == "wide_resnet50_2":
                self.model = models.wide_resnet50_2(pretrained=param_pretrained)
            elif model == "mnasnet":
                self.model = models.mnasnet1_0(pretrained=param_pretrained)
            interval = rospy.get_time() - start_time
            rospy.loginfo("Done load modle! Use {:.2f} seconds.".format(interval))
        else:
            rospy.loginfo("Your classifier is wrong. Please check out image label!")

    def use_cuda(self,cuda: bool=False):
        if cuda == True:
            rospy.loginfo("Using cuda! need some time to start...")
            device = torch.device('cuda')
            start_time = rospy.get_time()
            self.model = self.model.to(device)
            interval = rospy.get_time() - start_time
            rospy.loginfo("Done start. Can use cuda now! Use {:.2f} seconds.".format(interval)) 
        else:
            rospy.loginfo("Do not use cuda!")
    
    def train(self,epochs: int=30, best_model_path: str="best_model", _lr: float=0.001, _momentum: float=0.9):
        

    def getFilePath(self , name):
        rospack = rospkg.RosPack()
        return rospack.get_path('object') + "/image/" + name   

    def on_shutdown(self): 
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def read_param_from_file(self):
        fname = rospkg.RosPack().get_path('object') + "/param/image_label.yaml"
        folder = os.listdir(rospkg.RosPack().get_path('object')+"/image")
        with open(fname, 'r') as in_file:
            try:
                self.yaml_dict = yaml.load(in_file)
                for key in list(self.yaml_dict.keys()) :
                    if key not in folder :
                        rospy.loginfo("Please checkout folder [image] and label in [/param/image_label.yaml]. They are different.")
                        rospy.loginfo("train_model.py will shutdown.")
                        sys.exit()
                self.kind_of_classifier = len(list(self.yaml_dict.keys()))
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        if self.yaml_dict != None: 
            for label_name in self.yaml_dict:
                image_count = 0
                for dir_path, dir_names, file_names in os.walk(self.getFilePath(label_name)+ "/"):                   
                    for image in file_names:
                        if image.endswith('jpg') or image.endswith('jpeg') :
                            image_count += 1  
                self.yaml_dict[label_name] = image_count
        else:
            rospy.loginfo("Please use  {}  to make the floder for saving data ".format(rospkg.RosPack().get_path('object') + "/script/mkdir.py"))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == "__main__" :
    rospy.init_node("train_model",anonymous=False)
    train_model_node = Train_Model_Node()
    rospy.on_shutdown(train_model_node.on_shutdown)
    rospy.spin()
    
    
