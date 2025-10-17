import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory

def main():
    image_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
    print(image_path)
    image = cv2.imread(image_path)
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=2, model="hog")
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0,0,255), 2)
    cv2.imshow("Face Detection", image)
    cv2.waitKey(0)

