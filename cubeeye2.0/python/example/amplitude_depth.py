import queue
import sys
import getopt
import CubeEye as cu
import cv2
import numpy as np
import ctypes


amplitude_queue = queue.Queue()
depth_queue = queue.Queue()


class _CubeEyePythonSink(cu.Sink):
    def __init__(self):
        cu.Sink.__init__(self)

    def name(self):
        return "_CubeEyePythonSink"

    def onCubeEyeCameraState(self, name, serial_number, uri, state):
        _src = "(" + name + "/" + serial_number + ")"
        print("source:", _src + ", state:", state)

    def onCubeEyeCameraError(self, name, serial_number, uri, error):
        _src = "(" + name + "/" + serial_number + ")"
        print("source:", _src + ", error:", error)

    def onCubeEyeFrameList(self, name, serial_number, uri, frames):
        if frames is not None:
            for _frame in frames:
                if _frame.isBasicFrame():
                    if cu.DataType_U16 == _frame.dataType():
                        _u16_frame = cu.frame_cast_basic16u(_frame)
                        _u16_data_ptr = ctypes.c_uint16 * _u16_frame.dataSize()
                        _u16_data_ptr = _u16_data_ptr.from_address(int(_u16_frame.dataPtr()))
                        _u16_data_arr = np.ctypeslib.as_array(_u16_data_ptr)

                        # image draw with opencv
                        if cu.FrameType_Depth == _u16_frame.frameType():
                            # Depth Image
                            _u8_bgr = np.array(np.zeros(shape=(_frame.height(), _frame.width(), 3)), dtype=np.uint8)
                            cu.convert2bgr(_u16_data_arr, _u8_bgr)
                            # add to depth queue
                            depth_queue.put(_u8_bgr)
                        elif cu.FrameType_Amplitude == _u16_frame.frameType():
                            # Amplitude Image
                            _u8_gray = np.array(np.zeros(shape=(_frame.height(), _frame.width())), dtype=np.uint8)
                            cu.convert2gray(_u16_data_arr, _u8_gray)
                            # add to amplitude queue
                            amplitude_queue.put(_u8_gray)


if __name__ == "__main__":
    print("Hello CubeEye!")

    _selected_camera = ""
    _selected_camera_idx = 0

    # parse option arguments
    if 1 < len(sys.argv):
        print("input arguments:", sys.argv)
        try:
            opts, args = getopt.getopt(sys.argv[1:], "c:", ["camera="])
        except getopt.GetoptError as err:
            print("opt parse error : ", str(err))
            sys.exit(1)

        for opt, arg in opts:
            print("opt:", opt, ", arg:", arg)
            if "-c" == opt or "--camera":
                _selected_camera = arg

    # search CubeEye camera
    _source_list = cu.search_camera_source()
    if _source_list is None or 0 > _source_list.size():
        print("not found CubeEye camera!")
        sys.exit(1)
    print(_source_list)

    # check selected camera index
    if "" != _selected_camera:
        print("_selected_camera is ", _selected_camera)
        _list_size = _source_list.size()
        for idx in range(0, _list_size):
            if _source_list[idx].name() == _selected_camera:
                _selected_camera_idx = idx

    # create a camera
    _camera = cu.create_camera(_source_list[_selected_camera_idx])
    if _camera is None:
        print("create camera failed(source:", _source_list[_selected_camera_idx] + ")")
        sys.exit(1)

    # create sink object & add sink to camera
    _sink = _CubeEyePythonSink()
    if _sink is None:
        print("_CubeEyePythonSink is null!")
        sys.exit(1)
    _camera.addSink(_sink)

    # prepare camera
    _rt = _camera.prepare()
    if _rt is not cu.Result_Success:
        print("prepare failed:", _rt)
        cu.destroy_camera(_camera)
        sys.exit(1)

    # properties
    # set frame rate to 7
    # _camera.setProperty(cu.make_property_8u("framerate", 7))

    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Amplitude', cv2.WINDOW_AUTOSIZE)

    # run camera
    _rt = _camera.run(6)  # run with PCL + Amplitude
    if _rt is not cu.Result_Success:
        print("run failed:", _rt)
        cu.destroy_camera(_camera)
        sys.exit(1)

    # draw amplitude and depth
    while True:
        if amplitude_queue.empty() is False:
            amplitude = amplitude_queue.get_nowait()
            cv2.imshow("Amplitude", amplitude)
        if depth_queue.empty() is False:
            depth = depth_queue.get_nowait()
            cv2.imshow("Depth", depth)

        # wait for ESC key
        k = cv2.waitKey(1)
        if k == 27:
            break

    # stop camera
    _rt = _camera.stop()
    if _rt is not cu.Result_Success:
        print("stop failed:", _rt)
        cu.destroy_camera(_camera)
        sys.exit(1)

    # destroy camera
    cu.destroy_camera(_camera)
    print("bye bye~")

