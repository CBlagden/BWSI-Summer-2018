#!/usr/bin/env python
#Mayank Mali BWSI 2018

import argparse, cv2, os, sys, json
import numpy as np

DIRECTORY = None
SAVE_DATA_PATH = None
IMG_WIDTH = None
IMG_HEIGHT = None

datasets = []
metadata = {"total_images":0, "categories":[], "counts":{}, "label_map":{}, "height":-1, "width":-1}

def parse_args():
    global DIRECTORY
    global SAVE_DATA_PATH
    global IMG_WIDTH
    global IMG_HEIGHT


    parser = argparse.ArgumentParser(description="Converts a set of images into a JSON file. Adds gaussian noise to data. Written by Mayank Mali 2018.")
    parser.add_argument("-d", "--directory", dest="directory", type=str, required=True, help="Super-directory containing directories of images.")
    parser.add_argument("-o", "--output", dest="output", type=str, required=True, help="JSON file to which data + metadata should be saved.")
    parser.add_argument("-s", "--size", dest="size", type=str, required=False, default=None, help="dimensions (width,height) to resize images ex. --size 28x28. Default is to keep original dimensions.")
    args = parser.parse_args()

    DIRECTORY = args.directory
    SAVE_DATA_PATH = args.output

    if(args.size != None):
        IMG_WIDTH, IMG_HEIGHT = args.size.split("x")
        IMG_WIDTH = int(IMG_WIDTH)
        IMG_HEIGHT = int(IMG_HEIGHT)
        metadata["height"] = IMG_HEIGHT
        metadata["width"] = IMG_WIDTH


def load_data():


    assert DIRECTORY != None, "Directory is None. Maybe parse_args() has not been called yet?"
    
    global datasets
    global metadata

    directories = os.listdir(DIRECTORY)
    directories = list(filter(lambda i: os.path.isdir(os.path.join(DIRECTORY, i)), directories))

    print("directories : images")
    for d in directories:
        print("\t%s : %d" % (d, len(os.listdir(os.path.join(DIRECTORY, d)))))
    
    print("loading data ...")
    #for each subdirectory, find all images
    for direc in directories:
        
        #list all images in directory
        images = os.listdir(os.path.join(DIRECTORY, direc))
        
        metadata["counts"].update({direc:0})
        metadata["categories"].append(direc)

        #generate one-hot label
        label = [1 if directories[i] == direc else 0 for i in range(0,len(directories))]
        metadata["label_map"].update({direc:label})

        #print 000% progress
        form = direc + " : %03d%%"
        sys.stdout.write(form % 0)

        #for each image, load into datasets
        for i in range(0, len(images)):

            img = images[i]

            #read and resize the grayscaled image
            img_gray = cv2.imread(os.path.join(DIRECTORY, direc, img), cv2.IMREAD_GRAYSCALE)

            if(IMG_WIDTH != None and IMG_HEIGHT != None):
                img_gray = cv2.resize(img_gray, (IMG_WIDTH, IMG_HEIGHT))
            
            #add severe gaussian noise to images
            noise = np.random.normal(np.random.randint(-100, 100), 20, img_gray.shape)
            
            #make sure grayscale image pixel values are between 0, 255
            img_gray = np.clip(img_gray + noise, 0, 255)

            #add to number of images for that subdirectory
            metadata["counts"][direc] += 1
            
            #add data to datasets
            datasets.append({"data":img_gray.tolist(), "label":label, "name":direc})
            
            #print progress
            
            prog_print = form % int(100.0 * float(i + 1) / len(images))
            sys.stdout.write("\b" * len(prog_print))
            sys.stdout.write(prog_print)
            sys.stdout.flush()
        
        sys.stdout.write("\n")

        #update metadata total image count
        metadata["total_images"] += metadata["counts"][direc]

    print("saving data as JSON...")
    
    output = {"dataset":datasets, "metadata":metadata}

    with open(SAVE_DATA_PATH, "w") as outfile:
        json.dump(output, outfile, indent=2)


parse_args()
load_data()

