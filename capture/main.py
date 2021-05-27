import numpy as np
import cv2 as cv
import time as time
import vertexDetection
import disparityMap
import functools
import open3d as o3d
import reprojection
import sys

EPS = 13000000
USER = "USER"
DEV = "DEV"
MODE = USER


def combine_dims(a, i=0, n=1):
  s = list(a.shape)
  combined = functools.reduce(lambda x, y: x*y, s[i:i+n+1])
  return np.reshape(a, s[:i] + [combined] + s[i+n+1:])


def equal(mat_a, mat_b):
    if mat_a.shape != mat_b.shape:
        return False
    diff = np.sum(mat_a - mat_b, axis=0)
    diff = np.sum(diff, axis=0)
    return diff[0] <= EPS or diff[1] <= EPS or diff[2] <= EPS


def nothing(x):
    pass


if __name__ == "__main__":
    url = "http://192.168.0.111:81/stream"

    if len(sys.argv) > 1 and sys.argv[1] == "-dev":
        MODE = DEV
        print("You are running developer mode")

    if MODE == USER:
        while True:
            print("Provide your IP camera url (eg. http://192.168.0.1:81/stream) "
                  "or choose existing camera from your computer (eg. 0, 1)")

            url = input()
            print("Exit by pressing ESC while focused on camera window")
            print("Loading...")

            if url.isnumeric():
                url = int(url)

            camera = cv.VideoCapture(url)
            if camera is None or not camera.isOpened():
                print("WRONG camera url or IP provided")
            else:
                break
    else:
        print("Provide your IP camera url (eg. http://192.168.0.1:81/stream) or "
              "choose existing camera from your computer (eg. 0, 1)")
        url = input()
        if url.isnumeric():
            url = int(url)

        print("Exit by pressing ESC while focused on camera window")
        print("Loading...")
        camera = cv.VideoCapture(url)

    vertex_window_name = "Vertex test"
    disparity_window_name = "Disparity test"
    left_window_name = "Camera"
    right_window_name = "Right"
    settings_window_name = "Settings"

    all_points = np.ndarray(shape=(0, 3), dtype=float)
    all_colors = np.ndarray(shape=(0, 3), dtype=float)

    params = {}
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    added = False
    pcd = o3d.geometry.PointCloud()

    params["numDisparities"] = 1
    params["blockSize"] = 0
    params["filterSize"] = 15
    params["filterCap"] = 20
    params["minDisparity"] = 0
    params["textureThreshold"] = 0
    params["uniquenessRatio"] = 8
    params["speckleWindowSize"] = 79
    params["speckleRange"] = 51
    params["button"] = 0
    params["reset"] = 0
    params["save"] = 0

    cv.namedWindow(left_window_name, 0)
    if MODE == DEV:
        cv.namedWindow(vertex_window_name, 0)
        cv.namedWindow(disparity_window_name, 0)
        cv.namedWindow(right_window_name, 0)
        cv.namedWindow(settings_window_name, 0)

    ret, frame = camera.read()

    if not ret:
        print("NO FRAME!")
        exit(1)

    frame_right = np.copy(frame)
    frame_counter = 0
    travel_time = 0.0
    start = time.time()
    end = time.time()
    saved_models = 1

    def callback(name, val):
        params[name] = val

    cv.createTrackbar("PreFilterSize", settings_window_name, params["filterSize"], 256, lambda val: callback("filterSize", val))
    cv.createTrackbar("PreFilterCap", settings_window_name, params["filterCap"], 63, lambda val: callback("filterCap", val))
    cv.createTrackbar("NumDisparities", settings_window_name, params["numDisparities"], 256, lambda val: callback("numDisparities", val))
    cv.createTrackbar("BlockSize", settings_window_name, params["blockSize"], 256, lambda val: callback("blockSize", val))
    cv.createTrackbar("FilterSize", settings_window_name, params["filterSize"], 256, lambda val: callback("filterSize", val))
    cv.createTrackbar("MinDisparity", settings_window_name, params["minDisparity"], 256, lambda val: callback("minDisparity", val))
    cv.createTrackbar("TextureThreshold", settings_window_name, params["textureThreshold"], 256, lambda val: callback("textureThreshold", val))
    cv.createTrackbar("UniquenessRatio", settings_window_name, params["uniquenessRatio"], 256, lambda val: callback("uniquenessRatio", val))
    cv.createTrackbar("SpeckleWindowSize", settings_window_name, params["speckleWindowSize"], 256, lambda val: callback("speckleWindowSize", val))
    cv.createTrackbar("SpeckleRange", settings_window_name, params["speckleRange"], 256, lambda val: callback("speckleRange", val))

    while True:
        cv.createTrackbar("Scan", left_window_name, params["button"], 1, lambda val: callback("button", val))
        cv.createTrackbar("Reset", left_window_name, params["reset"], 1, lambda val: callback("reset", val))
        cv.createTrackbar("Save", left_window_name, params["save"], 1, lambda val: callback("save", val))
        start = end
        ret, frame = camera.read()
        if not ret:
            print("NO FRAME!")
            exit(1)

        if params["reset"] == 1:
            all_points = np.ndarray(shape=(0, 3), dtype=float)
            all_colors = np.ndarray(shape=(0, 3), dtype=float)
            pcd.points = o3d.utility.Vector3dVector(all_points)
            pcd.colors = o3d.utility.Vector3dVector(all_colors)
            params["reset"] = 0

        end = time.time()
        frame_left = np.copy(frame_right)
        frame_right = np.copy(frame)

        if params["button"] == 1 and not equal(frame_left, frame_right):
            travel_time += end - start
            vertices_frame = vertexDetection.vertex_detection(frame)
            disparity_frame = disparityMap.disparity_map(frame_left, frame_right, params)
            disparity_frame.fill(2)

            if MODE == DEV:
                cv.imshow(vertex_window_name, vertices_frame)
                cv.imshow(disparity_window_name, disparity_frame)

            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            center_color = frame[frame.shape[0] // 2, frame.shape[1] // 2]
            lower_bound = center_color - 10
            upper_bound = center_color + 10
            mask = cv.inRange(frame, lower_bound, upper_bound)

            points, colors = reprojection.reproject(disparity_frame, frame, travel_time, vertices_frame, mask)
            all_points = np.concatenate((all_points, points), axis=0)
            all_colors = np.concatenate((all_colors, colors), axis=0)

            pcd.points = o3d.utility.Vector3dVector(all_points)
            pcd.colors = o3d.utility.Vector3dVector(all_colors)

            if not added:
                added = True
                vis.add_geometry(pcd)
            else:
                vis.update_geometry(pcd)

        # print(f"Travel time: {travel_time}")
        cv.imshow(left_window_name, frame_left)

        if MODE == DEV:
            cv.imshow(right_window_name, frame_right)

        if params["save"] == 1:
            o3d.io.write_point_cloud("res" + str(saved_models) + ".ply", pcd)
            saved_models += 1
            params["save"] = 0

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break

        vis.poll_events()
        vis.update_renderer()

    camera.release()
    cv.destroyAllWindows()

