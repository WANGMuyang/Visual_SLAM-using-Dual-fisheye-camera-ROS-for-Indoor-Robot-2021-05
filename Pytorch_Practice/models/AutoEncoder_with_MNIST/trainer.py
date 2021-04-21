#!/usr/bin/env python
# coding: utf-8
from copy import deepcopy

import numpy as np

import torch
import torch.nn.functional as F
import torch.optim as optim


class Trainer():
    def __init__(self,model,optimizer,crit):
        self.model = model
        self.optimizer = optimizer
        self.crit = crit
        
        super().__init__()
    
    def _train(self,x,y,config):
        self.model.train() # 중요하다!
        
        #Shuffle before begin.
        indices = torch.randperm(x.size(0),device = x.device)
        x = torch.index_select(x,dim=0,index = indices).split(config.batch_size,dim=0)
        y = torch.index_select(y,dim=0,index = indices).split(config.batch_size,dim=0)
        
        total_loss = 0
        
        for i, (x_i,y_i) in enumerate(zip(x,y)):
            y_hat = self.model(x_i)
            loss_i = self.crit(y_hat,y_i.squeeze())
            
            self.optimizer.zero_grad()
            loss_i.backward()
            
            self.optimizer.step()
            
            if(config.verbose >= 2 ):
                print("Train iteration(%d/%d): loss = %.4e" % (i+1,len(x),float(loss_i)))
                
            total_loss += float(loss_i) # float를 적용 안해주면 loss_i는 tensor가 되서 memory leak 발생!!!
            
        return (total_loss/len(x))
    
    def _validate(self,x,y,config):
        self.model.eval() # 중요하다!
        
        with torch.no_grad():
            #Shuffle before begin.
            indices = torch.randperm(x.size(0),device = x.device)
            x = torch.index_select(x,dim=0,index = indices).split(config.batch_size,dim=0)
            y = torch.index_select(y,dim=0,index = indices).split(config.batch_size,dim=0)
        
            total_loss = 0

            for i, (x_i,y_i) in enumerate(zip(x,y)):
                y_hat = self.model(x_i)
                loss_i = self.crit(y_hat,y_i.squeeze())

                if(config.verbose >= 2 ):
                    print("Train iteration(%d/%d): loss = %.4e" % (i+1,len(x),float(loss_i)))
                    
                # float를 적용 안해주면 loss_i는 tensor가 되서 memory leak 발생!!!s
                total_loss += float(loss_i)

            return (total_loss/len(x))
    
    def train(self,train_data,valid_data,config):
        lowest_loss = np.inf
        best_model = None
        
        for epoch_index in range(config.n_epochs):
            train_loss = self._train(train_data[0],train_data[1],config)
            valid_loss = self._valid(valid_data[0],valid_data[1],config)
            
            if valid_loss<=lowest_loss:
                valid_loss = lowest_loss
                best_model = deepcopy(self.model.state_dict())
                
            print("Epoch (%d/%d): train_loss = %.4e valid_loss = %.4e lowest_loss=%.4e" % (
                epoch_index+1,
                config_n.epochs,
                train_loss,
                valid_loss,
                lowest_loss,
            ))
            
        self.model.load_state_dict(best_model)






