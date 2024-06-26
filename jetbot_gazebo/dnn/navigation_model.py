#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import shutil
import numpy as np

import torch
import torch.nn as nn

import torchvision.transforms as transforms
import torchvision.datasets as datasets
import torchvision.models as models

from torch.optim.lr_scheduler import StepLR, ReduceLROnPlateau
from datetime import datetime

from reshape_model import reshape_model
from xy_dataset import XYDataset


model_names = sorted(name for name in models.__dict__
    if name.islower() and not name.startswith("__")
    and callable(models.__dict__[name]))

torch.manual_seed(1)  # reproducibility


class NavigationModel:
    """
    Model for navigation
    """
    def __init__(self, model, type='regression', resolution=224, warmup=5):
        """
        Create or load a model.
        """
        if type != 'classification' and type != 'regression':
            raise ValueError("type must be 'classification' or 'regression' (was '{0}')".format(type))
            
        self.type = type
        self.resolution = resolution
        
        self.data_transforms = transforms.Compose([
                transforms.Resize((self.resolution, self.resolution)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225]),
            ])
            
        # load model
        if len(os.path.splitext(model)[1]) > 0:
            print("=> loading model '{0}'".format(model))
            checkpoint = torch.load(model, map_location=torch.device('cpu'))
            
            if self.type != checkpoint['type']:
                raise ValueError("'{0}' is a {1} model, but expected a {2} model".format(model, checkpoint['type'], self.type))
                
            print("     - arch           {0}".format(checkpoint['arch']))
            print("     - type           {0}".format(checkpoint['type']))
            print("     - outputs        {0}".format(checkpoint['num_outputs']))
            print("     - train loss     {0:.8f}".format(checkpoint['train_loss']))
            print("     - val loss       {0:.8f}".format(checkpoint['val_loss']))
            
            if self.classification:
                print("     - train accuracy {0:.8f}".format(checkpoint['train_accuracy']))
                print("     - val accuracy   {0:.8f}".format(checkpoint['val_accuracy']))

            self.model_arch = checkpoint['arch']
            self.num_outputs = checkpoint['num_outputs']
            
            self.model = models.__dict__[self.model_arch](pretrained=False)
            self.model = reshape_model(self.model, self.model_arch, self.num_outputs)
            
            self.model.load_state_dict(checkpoint['state_dict'])
        else:
            print("=> creating model '{0}'".format(model))
            self.model = models.__dict__[model](pretrained=True)
            self.model_arch = model
            self.num_outputs = 1000    # default classes for torchvision models
    
        # warmup model inference
        print("=> running model warm-up")
        
        for i in range(warmup):
            self.model.eval()
            
            with torch.no_grad():
                input = torch.ones((1, 3, resolution, resolution))
                output = self.model(input)
        
        print("=> done with model warm-up")
        
    @property
    def classification(self):
        return self.type == 'classification'
      
    @property
    def regression(self):
        return self.type == 'regression'
    
    def infer(self, image):
        # Ensure the image is converted and transformed properly
        image = self.data_transforms(image).unsqueeze(0)

        # Set the model to evaluation mode
        self.model.eval()

        with torch.no_grad():
            output = self.model(image)

            if self.classification:
                # Apply softmax to get probabilities for classification
                output = nn.functional.softmax(output, dim=1)
                prob, cls = torch.max(output, 1)
                return cls.item(), prob.item()
            else:
                # For regression, return the output directly
                return output.detach().squeeze().numpy() if output.requires_grad else output.squeeze().numpy()


    def load_dataset(self, dataset, batch_size=2, workers=1, train_split=0.8):
        """
        Load dataset from the specified path
        """
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
                            
        if self.type == 'classification':
            dataset = datasets.ImageFolder(dataset, self.data_transforms)
        elif self.type == 'regression':
            dataset = XYDataset(dataset, self.data_transforms)
            
        # split into train/val
        if train_split > 0:
            lengths = [int(len(dataset) * train_split)]
            lengths.append(len(dataset) - lengths[0])

            train_dataset, val_dataset = torch.utils.data.random_split(dataset, lengths)
                
            val_loader = torch.utils.data.DataLoader(
                val_dataset, batch_size=batch_size, num_workers=workers,
                shuffle=False, pin_memory=True)
        else:
            train_dataset = dataset
            val_dataset = None
            val_loader = None
            
        # create dataloaders
        train_loader = torch.utils.data.DataLoader(
            train_dataset, batch_size=batch_size, num_workers=workers,
            shuffle=True, pin_memory=True)

        print('=> train samples:   {0}'.format(len(train_dataset)))
        print('=> val samples:     {0}'.format(len(val_dataset) if val_dataset is not None else 0))
        
        # reshape model if needed   
        if self.type == 'classification':
            num_outputs = len(dataset.classes)
            print('=> dataset classes: {0} ({1})'.format(num_outputs, str(dataset.classes)))
        else:
            num_outputs = 2
            
        if self.num_outputs != num_outputs:
            self.model = reshape_model(self.model, self.model_arch, num_outputs)
            self.num_outputs = num_outputs

        # get class weights
        if self.type == 'classification':
            class_weights, class_counts = self.get_class_weights(dataset)
            
            print('=> class distribution:')
            
            for idx, (weight, count) in enumerate(zip(class_weights, class_counts)):
                print('     [{0}] - {1} samples ({2:.4f}), weight {3:.8f}'.format(idx, count, count/sum(class_counts), weight))
        else:
            class_weights = [1.0] * self.num_outputs
            
        return train_loader, val_loader, class_weights
        
    def train(self, dataset, epochs=10, batch_size=1, learning_rate=0.01, scheduler='StepLR_75', 
              workers=1, train_split=0.8, print_freq=10, use_class_weights=True, 
              save="data/models/{0}".format(datetime.now().strftime('%Y%m%d%H%M'))):
        """
        Train the model on a dataset
        """
        train_loader, val_loader, class_weights = self.load_dataset(dataset, batch_size, workers, train_split)
        
        # setup model, loss function, and solver
        if self.classification:
            criterion = nn.CrossEntropyLoss(weight=torch.Tensor(class_weights) if use_class_weights else None)
        else:
            criterion = nn.MSELoss()
            
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        scheduler = self._create_scheduler(scheduler, optimizer)        
        best_metric = -np.inf if self.classification else np.inf
        
        # train for the specified number of epochs
        for epoch in range(epochs):
            self.model.train()
            
            train_loss = 0.0
            train_accuracy = 0.0
            
            for i, (images, target) in enumerate(train_loader):
                # Ensure tensors are on CPU
                images = images
                target = target
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # compute gradient and do solver step
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        
                # keep track of accuracy/loss over the batch
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                train_accuracy += accuracy
                train_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print("Epoch {0}:  train=[{1}/{2}]  lr={3:.2g}  loss={4:.8f}  accuracy={5:.8f}".format(epoch, i, len(train_loader), scheduler.get_last_lr()[0], train_loss/(i+1), train_accuracy/(i+1)))
                    else:
                        print("Epoch {0}:  train=[{1}/{2}]  lr={3:.2g}  loss={4:.8f}".format(epoch, i, len(train_loader), scheduler.get_last_lr()[0], train_loss/(i+1)))
  
            if isinstance(scheduler, ReduceLROnPlateau):
                scheduler.step(metrics=train_loss)
            else:
                scheduler.step()
                
            train_loss /= len(train_loader)
            train_accuracy /= len(train_loader)
                    
            if val_loader is not None:
                val_loss, val_accuracy = self.validate(val_loader, criterion, epoch, print_freq)
            else:
                val_loss = train_loss
                val_accuracy = train_accuracy
                
            if self.classification:
                print("Epoch {0}:  train_loss={1:.8f}  train_accuracy={2:.8f}".format(epoch, train_loss, train_accuracy))
                print("Epoch {0}:  val_loss={1:.8f}  val_accuracy={2:.8f}".format(epoch, val_loss, val_accuracy))
            else:
                print("Epoch {0}:  train_loss={1:.8f}".format(epoch, train_loss))
                print("Epoch {0}:  val_loss={1:.8f}".format(epoch, val_loss))
                
            if save:
                checkpoint = {
                    'epoch': epoch,
                    'arch': self.model_arch,
                    'type': self.type,
                    'resolution': self.resolution,
                    'num_outputs': self.num_outputs,
                    'state_dict': self.model.state_dict(),
                    'train_loss': train_loss.item(),
                    'val_loss': val_loss.item(),
                }
                
                if self.classification:
                    checkpoint['train_accuracy'] = train_accuracy
                    checkpoint['val_accuracy'] = val_accuracy
                    
                is_best = val_accuracy > best_metric if self.classification else val_loss < best_metric
                self.save_checkpoint(checkpoint, is_best, save)
            
            if self.classification:
                best_metric = max(val_accuracy, best_metric)
            else:
                best_metric = min(val_loss, best_metric)
                
    def validate(self, val_loader, criterion, epoch, print_freq=10):
        """
        Measure model performance on the val dataset
        """
        self.model.eval()
        
        val_loss = 0.0
        val_accuracy = 0.0
        
        with torch.no_grad():
            for i, (images, target) in enumerate(val_loader):
                # Ensure tensors are on CPU
                images = images
                target = target
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # update accuracy and loss
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                val_accuracy += accuracy
                val_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print("Epoch {0}:  val=[{1}/{2}]  loss={3:.8f}  accuracy={4:.8f}".format(epoch, i, len(val_loader), val_loss/(i+1), val_accuracy/(i+1)))
                    else:
                        print("Epoch {0}:  val=[{1}/{2}]  loss={3:.8f}".format(epoch, i, len(val_loader), val_loss/(i+1)))
                    
        val_loss /= len(val_loader)
        val_accuracy /= len(val_loader)
        
        return val_loss, val_accuracy
        
    def accuracy(self, output, target):
        """
        Compute the classification accuracy.
        """
        _, preds = torch.max(output, 1)
        return (preds == target).float().mean().cpu().item()            
    
    def save_checkpoint(self, state, is_best, path=None, filename='checkpoint.pth', best_filename='model_best.pth'):
        """
        Save a model checkpoint file, along with the best-performing model if applicable
        """
        if path:
            filename = os.path.join(path, filename)
            best_filename = os.path.join(path, best_filename)
            
        dirname = os.path.dirname(filename)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        
        best_dirname = os.path.dirname(best_filename)
        if not os.path.exists(best_dirname):
            os.makedirs(best_dirname)
 
        # save the checkpoint
        torch.save(state, filename)

        # earmark the best checkpoint
        if is_best:
            shutil.copyfile(filename, best_filename)
            print("saved best model to:  " + best_filename)
        else:
            print("saved checkpoint to:  " + filename)

    @staticmethod
    def _create_scheduler(scheduler, optimizer):
        """
        Create a scheduler from a param string like 'StepLR_30'
        """
        if scheduler.startswith('StepLR'):
            return StepLR(optimizer, step_size=NavigationModel._parse_param(scheduler, default=30))
        elif scheduler.startswith('ReduceLROnPlateau'):
            return ReduceLROnPlateau(optimizer, patience=NavigationModel._parse_param(scheduler, default=10))
        else:
            raise ValueError("invalid scheduler '{0}'".format(scheduler)) 
        
    @staticmethod
    def _parse_param(str, default):
        """
        Parse a parameter in a string of the form 'text_value'
        """
        idx = str.find('_')
        
        if idx < 0 or idx == (len(str) - 1):
            return default

        return int(str[idx+1:])