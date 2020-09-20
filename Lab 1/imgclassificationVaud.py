#!/usr/bin/env python

##############
#### Your name:
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import data, io, feature, filters, exposure, color, exposure
from skimage.feature import hog
from skimage.filters import median
from skimage.color import rgb2gray
from skimage.transform import resize
from skimage.measure import LineModelND, ransac
import ransac_score

class ImageClassifier:
    
    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        ########################
        ######## YOUR CODE HERE
        ########################
        fd = None
        for img in data:
            grayscale = rgb2gray(img)
            med_blur = median(grayscale)
            #adapt_hist_eq = exposure.equalize_adapthist(med_blur)
            #edges = feature.canny(edges)
            features, hog_image = hog(med_blur, orientations=16, pixels_per_cell=(16, 16),
                        cells_per_block=(1, 1), visualize=True)
            # Please do not modify the return type below
            feat = np.array(features)
            feat = feat.reshape(1,-1)
            if fd is None:
                fd = feat#[None,:]
            else:
                fd = np.vstack((fd, feat))
        return(fd)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        # train model and save the trained model to self.classifier
        
        ########################
        ######## YOUR CODE HERE
        ########################
        self.classifier = svm.SVC()
        self.classifier.fit(train_data, train_labels)


    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels
        
        ########################
        ######## YOUR CODE HERE
        ########################

        # Please do not modify the return type below
        predicted_labels = []
        for feature_set in data:
            predicted_labels.append(self.classifier.predict(feature_set[None,:]))
        return predicted_labels

    def line_fitting(self, data):
        # Please do not modify the header

        # fit a line the to arena wall using RANSAC
        # return two lists containing slopes and y intercepts of the line

        ########################
        ######## YOUR CODE HERE
        ########################

        # Please do not modify the return type below
        slope = []
        intercept = []
        for img in data:
            grayscale = rgb2gray(img)
            grayscale = grayscale[0:100]
            edges = feature.canny(grayscale)
            points = np.where(edges)
            
            x = points[0]
            y=points[1]
            
            points = np.column_stack([y,x])
            model_robust, inliers = ransac(points, LineModelND, min_samples=2, residual_threshold=5, max_trials=1000)
            line_x = np.arange(0, 240)
            line_y_robust = model_robust.predict_y(line_x)
            i = line_y_robust[0]
            s = line_y_robust[1] - i
            slope.append(s)
            intercept.append(i)
        return slope, intercept
        
def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    (wall_raw, _) = img_clf.load_data_from_folder('./wall/')
    
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))
    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

    # ransac
    print("\nRANSAC results")
    print("=============================")
    s, i = img_clf.line_fitting(wall_raw)
    print(f"Line Fitting Score: {ransac_score.score(s,i)}/10")

if __name__ == "__main__":
    main()
