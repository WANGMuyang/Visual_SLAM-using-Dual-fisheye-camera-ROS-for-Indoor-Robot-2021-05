#!/usr/bin/env python
# coding: utf-8
import requests

def load_mnist(is_train = True,flatten = True):
    from torchvision import datasets,transforms
    
    dataset = datasets.MNIST(
        'C:/Users/Alpha/Desktop/Python_Workspace/data',train = is_train,download = False,
        transform = transforms.Compose([
            transforms.ToTensor(),
        ]),
    )
    
    x = dataset.data.float() / 255.
    y = dataset.targets
    
    if flatten:
        x = x.view(x.size(0),-1)
        
    return x,y

