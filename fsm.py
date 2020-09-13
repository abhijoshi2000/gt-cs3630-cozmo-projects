import joblib
import time
import sklearn
import cozmo
import random
from cozmo.util import degrees, distance_mm, speed_mmps
import numpy as np
from imgclassification_sol import ImageClassifier


def wait_for_state(robot: cozmo.robot.Robot):
    # Step 1: Enable camera stream and set up Cozmo camera to allow it to constantly scan for images
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Step 2: Process images and extract the state from the image using our image classification model
    image_classifier = joblib.load('trained_model_1.pkl')
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
            predictions.append(pred[0])

        state, freq = find_majority(predictions)
        print(state)

        # Step 3: Encode the proper behavior for every state

        # Mission 2
        if state == 'order':

            robot.say_text(
                "Order state detected. Starting Mission 2").wait_for_completed()

            lookaround = robot.start_behavior(
                cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            cubes = robot.world.wait_until_observe_num_objects(
                num=1, object_type=cozmo.objects.LightCube, timeout=60)
            lookaround.stop()

            pickup_cube = robot.pickup_object(cubes[0], num_retries=3)
            pickup_cube.wait_for_completed()

            drive_forward = robot.drive_straight(distance_mm(
                300.0), speed_mmps(50.0))
            drive_forward.wait_for_completed()

            place_on_ground = robot.place_object_on_ground_here(
                cubes[0], num_retries=3)
            place_on_ground.wait_for_completed()

            drive_backward = robot.drive_straight(distance_mm(-325.0),
                                                  speed_mmps(50.0))
            drive_backward.wait_for_completed()

            continue

        # Mission 3

        if state == 'drone':
            # Drive in S formation

            # Get and play animation
            all_animation_triggers = robot.anim_triggers
            random.shuffle(all_animation_triggers)
            triggers = 1
            chosen_trigger = all_animation_triggers[:triggers][0]
            robot.play_anim_trigger(chosen_trigger).wait_for_completed()

            continue

        # Mission 4

        if state == 'inspection':
            robot.say_text(
                "Inspection state detected. Starting Mission 4").wait_for_completed()

            for i in range(4):
                robot.say_text("I am not a spy!", in_parallel=True)
                drive_action = robot.drive_straight(distance_mm(
                    180), speed_mmps(50), in_parallel=True)
                robot.set_lift_height(
                    1.0, duration=2.0, in_parallel=True).wait_for_completed()
                robot.set_lift_height(
                    0.0, duration=2.0, in_parallel=True).wait_for_completed()
                drive_action.wait_for_completed()
                robot.turn_in_place(degrees(90)).wait_for_completed()

        continue


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
