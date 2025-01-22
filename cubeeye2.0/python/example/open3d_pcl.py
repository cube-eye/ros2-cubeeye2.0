import sys
import getopt
import CubeEye as cu
import numpy as np
import ctypes
import open3d as o3d
import threading

# global variables for point cloud with open3d
_o3d_vis_start = False
_o3d_vis_evt = threading.Event()
_o3d_vis_lock = threading.Lock()

_cur_pcl_x_data = np.array(np.zeros(shape=(480, 640)), dtype=np.float32)
_cur_pcl_y_data = np.array(np.zeros(shape=(480, 640)), dtype=np.float32)
_cur_pcl_z_data = np.array(np.zeros(shape=(480, 640)), dtype=np.float32)


def o3d_vis_escape_key_callback(vis):
    print("Escape!")
    global _o3d_vis_start
    _o3d_vis_start = False


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
        global _o3d_vis_evt, _o3d_vis_lock
        global _cur_pcl_x_data, _cur_pcl_y_data, _cur_pcl_z_data

        if frames is not None:
            for _frame in frames:
                if cu.FrameType_PointCloud == _frame.frameType():
                    if cu.DataType_F32 == _frame.dataType():
                        # Point Cloud Image
                        _o3d_vis_lock.acquire()
                        _f32_frame = cu.frame_cast_pcl32f(_frame)
                        _f32_data_x_ptr = ctypes.c_float * _f32_frame.dataXsize()
                        _f32_data_x_ptr = _f32_data_x_ptr.from_address(int(_f32_frame.dataXptr()))
                        _f32_data_x_arr = np.ctypeslib.as_array(_f32_data_x_ptr)

                        _f32_frame = cu.frame_cast_pcl32f(_frame)
                        _f32_data_y_ptr = ctypes.c_float * _f32_frame.dataYsize()
                        _f32_data_y_ptr = _f32_data_y_ptr.from_address(int(_f32_frame.dataYptr()))
                        _f32_data_y_arr = np.ctypeslib.as_array(_f32_data_y_ptr)

                        _f32_frame = cu.frame_cast_pcl32f(_frame)
                        _f32_data_z_ptr = ctypes.c_float * _f32_frame.dataZsize()
                        _f32_data_z_ptr = _f32_data_z_ptr.from_address(int(_f32_frame.dataZptr()))
                        _f32_data_z_arr = np.ctypeslib.as_array(_f32_data_z_ptr)

                        _cur_pcl_x_data = np.array(_f32_data_x_arr)
                        _cur_pcl_y_data = np.array(_f32_data_y_arr)
                        _cur_pcl_z_data = np.array(_f32_data_z_arr)

                        _o3d_vis_lock.release()
                        _o3d_vis_evt.set()


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

    # prepare point cloud with open3d
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    _o3d_vis = o3d.visualization.VisualizerWithKeyCallback()
    _o3d_vis.create_window(window_name="CubeEye PCL", height=480, width=640)

    # render option
    _opt = _o3d_vis.get_render_option()
    _opt.background_color = np.asarray([0, 0, 0])
    _opt.point_size = 1.0

    # see the key defines : https://www.glfw.org/docs/latest/group__keys.html
    _o3d_vis.register_key_callback(256, o3d_vis_escape_key_callback)  # GLFW_KEY_ESCAPE

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

    # run camera
    _rt = _camera.run(32)  # run with PCL + Amplitude
    if _rt is not cu.Result_Success:
        print("run failed:", _rt)
        cu.destroy_camera(_camera)
        sys.exit(1)

    _o3d_vis_start = True
    _o3d_vis_first_pcl_frame = True
    _o3d_pcl = o3d.geometry.PointCloud()

    while _o3d_vis_start is True:
        _o3d_vis_evt.wait()
        _o3d_vis_lock.acquire()
        _xyz_coordinate = np.transpose(np.vstack((_cur_pcl_x_data, _cur_pcl_y_data, _cur_pcl_z_data)))

        _o3d_vis_lock.release()
        _o3d_pcl.points = o3d.utility.Vector3dVector(np.negative(_xyz_coordinate))

        if _o3d_vis_first_pcl_frame is True:
            _o3d_vis_first_pcl_frame = False
            _o3d_vis.add_geometry(_o3d_pcl)
        else:
            _o3d_vis.update_geometry(_o3d_pcl)
        _o3d_vis.poll_events()
        _o3d_vis.update_renderer()

    # stop camera
    _rt = _camera.stop()
    if _rt is not cu.Result_Success:
        print("stop failed:", _rt)
        cu.destroy_camera(_camera)
        sys.exit(1)

    # destroy camera
    cu.destroy_camera(_camera)
    _o3d_vis.destroy_window()
    print("bye bye~")
    sys.exit(0)
