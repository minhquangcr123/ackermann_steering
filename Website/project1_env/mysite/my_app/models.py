from django.db import models

# Create your models here.

PRIORITY = [("L", "Low"), ("M", "Meidum"), ("H", "Height")]

class Questions(models.Model):
    def __init__(self):
        self.title =                    models.CharField(max_length = 60)
        self.question =                 models.TextField(max_length = 60)
        self.priority =                 models.CharField(max_length = 60, choices= PRIORITY)
    
    def __str__(self):
        return self.title