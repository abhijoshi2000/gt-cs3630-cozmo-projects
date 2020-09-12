import joblib
import cozmo
from cozmo.util import degrees


def wait_for_state(robot: cozmo.robot.Robot):
    # Step 1: Enable camera stream and set up Cozmo camera to allow it to constantly scan for images
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    robot.move_lift(-1)

    # Step 2: Process images and extract the state from the image using our image classification model
    while True:
        image_classifier = joblib.load('trained_model.pkl')

        # Step 3: Encode the proper behavior for every state


# Step 4: Write main method to run the wait_for_state function
def main():
    cozmo.run_program(wait_for_state).wait_for_completed()


if __name__ == '__main__':
    main()
