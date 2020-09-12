import joblib
import time
import sklearn
import cozmo
from cozmo.util import degrees
import numpy as np
from imgclassification_sol import ImageClassifier


def wait_for_state(robot: cozmo.robot.Robot):
    # Step 1: Enable camera stream and set up Cozmo camera to allow it to constantly scan for images
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Step 2: Process images and extract the state from the image using our image classification model
    image_classifier = joblib.load('trained_model.pkl')
    while True:
        predictions = []
        for i in range(10):
            time.sleep(0.1)
            latest_image = robot.world.latest_image
            new_image = latest_image.raw_image

            img = np.array(new_image)
            img = np.expand_dims(img, axis=0)
            feats = ImageClassifier.extract_image_features(0, img)
            pred = image_classifier.predict(feats)
            predictions.append(pred)
            print(pred)

        # Step 3: Encode the proper behavior for every state


def find_majority(k):
    myMap = {}
    maximum = ('', 0)  # (occurring element, occurrences)
    for n in k:
        if n in myMap:
            myMap[n] += 1
        else:
            myMap[n] = 1

        # Keep track of maximum on the go
        if myMap[n] > maximum[1]:
            maximum = (n, myMap[n])

    return maximum

# Step 4: Write main method to run the wait_for_state function


def main():
    cozmo.run_program(wait_for_state)


if __name__ == '__main__':
    main()
