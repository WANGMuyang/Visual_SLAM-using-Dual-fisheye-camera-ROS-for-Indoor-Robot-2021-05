import numpy as np
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.optim as optim

from utils import load_mnist
from trainer import Trainer

from argparse import Namespace

config = {
    'train ratio' : 0.8,
    'batch_size' : 256,
    'n_epochs' : 50,
    'verbose' : 1,
    'btl_size' : 2
} 

config  = Namespace(**config)

print(config)

train_x,train_y = load_mnist(flatten=True)
test_x,test_y = load_mnist = load_mnist(is_train=False,flatten=True)

train_cnt = int(train_x.size(0)*config.train_ratio)
valid_cnt = int(train_x.size(0)*(1-config.train_ratio))

indices = torch.randperm(train_x.size(0))

train_x,valid_x = torch.index_select(
    train_x,
    dim=0,
    index = indices
).to(device).split([train_cnt,valid_cnt],dim=0)

train_y,valid_y = torch.index_select(
    train_y,
    dim=0,
    index = indices
).to(device).split([train_cnt,valid_cnt],dim=0)

from AutoEncoder import autoencoder

model = autoencoder(btl_size = config.btl_size)
optimizer = optim.Adam(model.parameters())
crit = nn.MSELoss()

trainer = Trainer(model = model,optimizer=optimizer,crit=crit)

trainer.train((train_x,train_x),(valid_x,valid_x),config=config)


with torch.no_grad():
    import random

    index = int(random.random() * test_x.size(0))
    recon = model(test_x[index].view(1,-1)).squeeze()

    show_image(test_x[index])
    show_image(recon)