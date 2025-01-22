import CubeEye as cu
import numpy as np
import ctypes


def get_camera_state(state):
    _states = ('CameraState_Released', 'CameraState_Prepared', 'CameraState_Stopped', 'CameraState_Running')
    if state >= len(_states):
        return "Unknown State"
    return _states[state]


def get_camera_error(error):
    _errors = ('CameraError_Unknown', 'CameraError_IO', 'CameraError_AccessDenied', 'CameraError_NoSuchDevice',
               'CameraError_Busy', 'CameraError_Timeout', 'CameraError_Overflow', 'CameraError_Interrupted',
               'CameraError_Internal', 'CameraError_FrameDropped', 'CameraError_IlluminationLock',
               'CameraError_NoFramesReceived')
    if error >= len(_errors):
        return "Unknown error"
    return _errors[error]


def get_camera_result(result):
    _results = ('Result_Success', 'Result_Fail', 'Result_Empty', 'Result_Overflow',
                'Result_NotFound', 'Result_NotExist', 'Result_NotReady', 'Result_NotSupported',
                'Result_NotImplemented', 'Result_NotInitialized', 'Result_NoSuchDevice', 'Result_NoSuchFile',
                'Result_NoSuchNetworkInterface', 'Result_NoResponse', 'Result_InvalidParameter',
                'Result_InvalidOperation', 'Result_InvalidDataType', 'Result_InvalidChecksum',
                'Result_InvalidCompatibilityIndex', 'Result_OutOfMemory', 'Result_OutOfResource',
                'Result_OutOfRange', 'Result_AlreadyExists', 'Result_AlreadyOpened',
                'Result_AlreadyRunning', 'Result_AlreadyInitialized', 'Result_UsingResources', 'Result_Timeout')
    if result >= len(_results):
        return "Unknown result"
    return _results[result]


# camera sink class. it runs in another thread
class _CubeEyePythonSink(cu.Sink):
    def __init__(self):
        self.frame_count = 0
        cu.Sink.__init__(self)

    def name(self):
        return "_CubeEyePythonSink"

    def onCubeEyeCameraState(self, name, serial_number, uri, state):
        _src = "(" + name + "/" + serial_number + ")"
        print("source:", _src + ", state:", get_camera_state(state))

    def onCubeEyeCameraError(self, name, serial_number, uri, error):
        _src = "(" + name + "/" + serial_number + ")"
        print("source:", _src + ", error:", get_camera_error(error))

    def onCubeEyeFrameList(self, name, serial_number, uri, frames):
        if frames is not None:
            # process one frame every one second
            self.frame_count += 1
            if self.frame_count < 30:
                return
            self.frame_count = 0

            for _frame in frames:
                # get center position
                _center_x = _frame.width() // 2
                _center_y = _frame.height() // 2
                _position = _center_x * _frame.width() + _center_y

                if _frame.isBasicFrame():
                    if cu.DataType_U16 == _frame.dataType():
                        _u16_frame = cu.frame_cast_basic16u(_frame)
                        _u16_data_ptr = ctypes.c_uint16 * _u16_frame.dataSize()
                        _u16_data_ptr = _u16_data_ptr.from_address(int(_u16_frame.dataPtr()))
                        _u16_data_arr = np.ctypeslib.as_array(_u16_data_ptr)

                        if cu.FrameType_Depth == _u16_frame.frameType():
                            _data = _u16_data_arr[_position]
                            print(f'depth frame: position{_center_x, _center_y} data -> {_data}')
                        elif cu.FrameType_Amplitude == _u16_frame.frameType():
                            _data = _u16_data_arr[_position]
                            print(f'amplitude frame: position{_center_x, _center_y} data -> {_data}')
                elif cu.FrameType_PointCloud == _frame.frameType():
                    if cu.DataType_F32 == _frame.dataType():
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

                        _data_x = _f32_data_x_arr[_position]
                        _data_y = _f32_data_y_arr[_position]
                        _data_z = _f32_data_z_arr[_position]
                        print(f'PCL frame: position{_center_x, _center_y} data -> {_data_x}, {_data_y}, {_data_z}')


class _CubeEyeContext:
    def __init__(self):
        self.camera = None
        self.source_list = None
        self.sink = None


def search_command(context, command_list):
    # search CubeEye camera
    context.source_list = cu.search_camera_source()
    if context.source_list is None or 0 > context.source_list.size():
        print("not found CubeEye camera!")
    else:
        for source in enumerate(context.source_list):
            print(source)


def select_command(context, command_list):
    _selected_camera_idx = -1
    if len(command_list) < 2:
        print("wrong parameter")
        return

    if context.source_list is None or int(command_list[1]) >= context.source_list.size():
        print("source list is not valid")
        return

    _selected_camera_idx = int(command_list[1])

    # create a camera
    context.camera = cu.create_camera(context.source_list[_selected_camera_idx])
    if context.camera is None:
        print("create camera failed(source:", context.source_list[_selected_camera_idx] + ")")
        return

    # create sink object & add sink to camera
    context.sink = _CubeEyePythonSink()
    if context.sink is None:
        print("_CubeEyePythonSink is null!")
        return

    context.camera.addSink(context.sink)

    # prepare camera
    _rt = context.camera.prepare()
    if _rt is not cu.Result_Success:
        print("prepare failed:", _rt)
        cu.destroy_camera(context.camera)
        context.camera = None

    print("camera", str(_selected_camera_idx), "is selected")


def run_command(context, command_list):
    if len(command_list) < 2:
        _frame_type = cu.FrameType_Depth + cu.FrameType_Amplitude     # by default. Amplitude + Depth
    else:
        _frame_type = int(command_list[1])

    # run camera with frame type
    _rt = context.camera.run(_frame_type)
    if _rt is not cu.Result_Success:
        print("run failed:", get_camera_result(_rt))


def stop_command(context, command_list):
    _rt = context.camera.stop()
    if _rt is not cu.Result_Success:
        print("stop failed:", get_camera_result(_rt))


def set_command(context, command_list):
    if len(command_list) < 4:
        return

    _key = command_list[1]
    _dt = command_list[2]
    _value = command_list[3]

    _property = None
    if _dt == 'b':
        _property = cu.make_property_bool(_key, bool(_value))
    elif _dt == '8s':
        _property = cu.make_property_8s(_key, int(_value))
    elif _dt == '8u':
        _property = cu.make_property_8u(_key, int(_value))
    elif _dt == '16s':
        _property = cu.make_property_16s(_key, int(_value))
    elif _dt == '16u':
        _property = cu.make_property_16u(_key, int(_value))
    elif _dt == '32f':
        _property = cu.make_property_32f(_key, float(_value))

    if _property is None:
        print("data type is wrong:", _dt)
        return

    _rt = context.camera.setProperty(_property)
    if _rt is not cu.Result_Success:
        print("set", _property, " failed:", get_camera_result(_rt))


def get_command(context, command_list):
    if len(command_list) < 2:
        return

    _rt = context.camera.getProperty(command_list[1])
    if _rt[0] is not cu.Result_Success:
        print("get failed:", get_camera_result(_rt[0]))
    else:
        print("get:", _rt[1])


def help_command():
    print("Usage: >> [COMMAND]... [OPTION]")
    print("\tsearch\t\tsearching for ToF camera device.\t- e.g.: >> search")
    print("\tselect\t\tselect camera device in the searched list.\t- e.g.: >> select 0")
    print("\trun\t\t\trun the selected camera device with frame type.\t- e.g.: >> run 6")
    print("\tstop\t\tstop the camera device.\t- e.g.: >> stop")
    print("\tset\t\t\tset property.\t- e.g.: >> set framerate 8u 7")
    print("\tget\t\t\tget property.\t- e.g.: >> get framerate")
    print("\tquit\t\tquit this program.\t- e.g.: >> quit")


def main():
    _context = _CubeEyeContext()
    _command_dict = {"search": search_command, "select": select_command, "run": run_command, "stop": stop_command,
                     "set": set_command, "get": get_command}

    help_command()
    while True:
        _cmd_list = input(">> ").strip().split()
        if len(_cmd_list) == 0:
            continue
        if _cmd_list[0] == "quit":
            break

        if _cmd_list[0] in _command_dict:
            _command_dict[_cmd_list[0]](_context, _cmd_list)
        else:
            help_command()

    if _context.camera is not None:
        _rt = _context.camera.stop()

        print("destroy camera")
        cu.destroy_camera(_context.camera)

    print("bye bye~")


if __name__ == "__main__":
    main()
