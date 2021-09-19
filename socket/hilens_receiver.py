import cv2
from npsocket import NumpySocket

sock_receiver = NumpySocket()
sock_receiver.initalize_receiver('192.168.2.1', 9999)

if __name__ == '__main__':
    while True:
        conn, bboxes = sock_receiver.receive_array()
        if conn:
            for bbox in bboxes:
                label = int(bbox[4])
                x_min = int(bbox[0])
                y_min = int(bbox[1])
                x_max = int(bbox[2])
                y_max = int(bbox[3])
                score = bbox[5]
                curve_rad = bbox[6]
                distance_from_center = bbox[7]
                print('label: {0}, x_min: {1}, y_min: {2}, x_max: {3}, y_max:{4}, score:{5}, curve_rad: {6}, '
                      'distance_from_center: {7}\n'.format(label, x_min, y_min, x_max, y_max, score, curve_rad,
                                                           distance_from_center))
