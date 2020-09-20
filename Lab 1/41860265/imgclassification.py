#!/usr/bin/env python

##############
# Your name: Abhinav Joshi
##############

import matplotlib.pyplot as plt
import numpy as np
import re
from sklearn import svm, metrics, linear_model
from skimage import io, feature, filters, exposure, color, measure
import ransac_score


class ImageClassifier:

    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)

        # create one large array of image data
        data = io.concatenate_images(ic)

        # extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return(data, labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        ########################
        # YOUR CODE HERE
        feature_data = []
        for img in data:
            hog_image = feature.hog(img, orientations=10, pixels_per_cell=(
                24, 24), cells_per_block=(5, 5), block_norm='L2-Hys', multichannel=True)
            feature_data.append(hog_image)
        ########################
        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above

        # train model and save the trained model to self.classifier

        ########################
        # YOUR CODE HERE
        self.classifier = svm.SVC()
        self.classifier.fit(train_data, train_labels)
        print(self.classifier)
        ########################

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        ########################
        # YOUR CODE HERE
        predicted_labels = self.classifier.predict(data)
        ########################

        # Please do not modify the return type below
        return predicted_labels

    def line_fitting(self, data):
        # Please do not modify the header

        # fit a line the to arena wall using RANSAC
        # return two lists containing slopes and y intercepts of the line

        ########################
        # YOUR CODE HERE

        # Convert to grayscale and run Canny edge detection
        grayscale_edges = []
        for img in data:
            processed_img = color.rgb2gray(img)
            processed_img = feature.canny(processed_img, sigma=3.0)
            grayscale_edges.append(np.argwhere(processed_img))

        # Feed edges into the ransac function
        ransac_models = []
        for img in grayscale_edges:
            ransac_models.append(measure.ransac(
                img, min_samples=2, model_class=measure.LineModelND, residual_threshold=1)[0])

        # Calculate slope and intercept
        slope = []
        intercept = []
        index = 0
        for model in ransac_models:
            origin = model.params[0]
            direction = model.params[1]
            slope.append(direction[0] / direction[1])
            intercept.append((origin[0] - slope[index] * origin[1]))
            index += 1

        ########################

        # Please do not modify the return type below
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
    print("Confusion Matrix:\n", metrics.confusion_matrix(
        train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(
        train_labels, predicted_labels, average='micro'))

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(
        test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(
        test_labels, predicted_labels, average='micro'))

    # ransac
    print("\nRANSAC results")
    print("=============================")
    s, i = img_clf.line_fitting(wall_raw)
    print(f"Line Fitting Score: {ransac_score.score(s,i)}/10")


if __name__ == "__main__":
    main()
