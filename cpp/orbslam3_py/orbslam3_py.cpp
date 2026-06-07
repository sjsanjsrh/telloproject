#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include "slam/System.h"

namespace {

const char* state_name(int state)
{
    switch (state) {
    case -1: return "SYSTEM_NOT_READY";
    case 0: return "NO_IMAGES_YET";
    case 1: return "NOT_INITIALIZED";
    case 2: return "OK";
    case 3: return "RECENTLY_LOST";
    case 4: return "LOST";
    case 5: return "OK_KLT";
    default: return "UNKNOWN";
    }
}

PyObject* matrix_to_python(const cv::Mat& matrix)
{
    PyObject* rows = PyList_New(3);
    if (!rows) {
        return nullptr;
    }
    cv::Mat matrix64;
    matrix.convertTo(matrix64, CV_64F);
    for (int r = 0; r < 3; ++r) {
        PyObject* row = PyList_New(3);
        if (!row) {
            Py_DECREF(rows);
            return nullptr;
        }
        for (int c = 0; c < 3; ++c) {
            PyList_SET_ITEM(row, c, PyFloat_FromDouble(matrix64.at<double>(r, c)));
        }
        PyList_SET_ITEM(rows, r, row);
    }
    return rows;
}

PyObject* pose_to_dict(const cv::Mat& tcw, int tracking_state, double timestamp, int width, int height, long long frame_index)
{
    PyObject* result = PyDict_New();
    if (!result) {
        return nullptr;
    }

    PyDict_SetItemString(result, "timestamp", PyFloat_FromDouble(timestamp));
    PyDict_SetItemString(result, "tracking_state", PyUnicode_FromString(state_name(tracking_state)));
    PyDict_SetItemString(result, "tracking_state_id", PyLong_FromLong(tracking_state));
    PyDict_SetItemString(result, "image_width", PyLong_FromLong(width));
    PyDict_SetItemString(result, "image_height", PyLong_FromLong(height));
    PyDict_SetItemString(result, "frame_index", PyLong_FromLongLong(frame_index));

    if (tcw.empty()) {
        PyObject* position = Py_BuildValue("[d,d,d]", 0.0, 0.0, 0.0);
        PyObject* rotation = Py_BuildValue("[[d,d,d],[d,d,d],[d,d,d]]", 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        PyDict_SetItemString(result, "position", position);
        PyDict_SetItemString(result, "rotation_matrix", rotation);
        Py_DECREF(position);
        Py_DECREF(rotation);
        return result;
    }

    cv::Mat pose64;
    tcw.convertTo(pose64, CV_64F);
    cv::Mat twc = pose64.inv();
    PyObject* position = Py_BuildValue("[d,d,d]", twc.at<double>(0, 3), twc.at<double>(1, 3), twc.at<double>(2, 3));
    PyObject* rotation = matrix_to_python(twc.rowRange(0, 3).colRange(0, 3));
    PyDict_SetItemString(result, "position", position);
    PyDict_SetItemString(result, "rotation_matrix", rotation);
    Py_DECREF(position);
    Py_DECREF(rotation);
    return result;
}

typedef struct {
    PyObject_HEAD
    ORB_SLAM3::System* slam;
    long long frame_index;
} OrbSlam3MonoObject;

void OrbSlam3Mono_dealloc(OrbSlam3MonoObject* self)
{
    if (self->slam) {
        try {
            self->slam->Shutdown();
        } catch (...) {
        }
        delete self->slam;
        self->slam = nullptr;
    }
    Py_TYPE(self)->tp_free(reinterpret_cast<PyObject*>(self));
}

int OrbSlam3Mono_init(OrbSlam3MonoObject* self, PyObject* args, PyObject* kwargs)
{
    const char* vocab = nullptr;
    const char* settings = nullptr;
    int use_viewer = 0;
    static char* kwlist[] = {const_cast<char*>("vocab"), const_cast<char*>("settings"), const_cast<char*>("use_viewer"), nullptr};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|p", kwlist, &vocab, &settings, &use_viewer)) {
        return -1;
    }
    try {
        self->slam = new ORB_SLAM3::System(vocab, settings, ORB_SLAM3::System::MONOCULAR, use_viewer != 0);
        self->frame_index = 0;
    } catch (const std::exception& exc) {
        PyErr_SetString(PyExc_RuntimeError, exc.what());
        return -1;
    }
    return 0;
}

PyObject* OrbSlam3Mono_track(OrbSlam3MonoObject* self, PyObject* args)
{
    PyObject* frame_obj = nullptr;
    double timestamp = 0.0;
    if (!PyArg_ParseTuple(args, "Od", &frame_obj, &timestamp)) {
        return nullptr;
    }
    if (!self->slam) {
        PyErr_SetString(PyExc_RuntimeError, "ORB-SLAM3 system is not initialized");
        return nullptr;
    }

    Py_buffer view;
    if (PyObject_GetBuffer(frame_obj, &view, PyBUF_ND | PyBUF_STRIDES | PyBUF_FORMAT) != 0) {
        return nullptr;
    }

    PyObject* output = nullptr;
    try {
        if (view.itemsize != 1 || view.ndim < 2 || view.ndim > 3) {
            throw std::runtime_error("frame must be uint8 HxW or HxWxC");
        }
        if (view.format && view.format[0] != 'B' && view.format[0] != 'b') {
            throw std::runtime_error("frame must expose uint8 buffer format");
        }

        const int height = static_cast<int>(view.shape[0]);
        const int width = static_cast<int>(view.shape[1]);
        const int channels = view.ndim == 3 ? static_cast<int>(view.shape[2]) : 1;
        if (channels != 1 && channels != 3 && channels != 4) {
            throw std::runtime_error("frame channel count must be 1, 3, or 4");
        }
        if (view.strides[view.ndim - 1] != 1) {
            throw std::runtime_error("frame must be contiguous along channel dimension");
        }

        const int type = channels == 1 ? CV_8UC1 : channels == 3 ? CV_8UC3 : CV_8UC4;
        cv::Mat image(height, width, type, view.buf, static_cast<size_t>(view.strides[0]));
        cv::Mat contiguous = image.isContinuous() ? image : image.clone();

        ++self->frame_index;
        cv::Mat tcw;
        Py_BEGIN_ALLOW_THREADS
        tcw = self->slam->TrackMonocular(contiguous, timestamp);
        Py_END_ALLOW_THREADS

        const int tracking_state = self->slam->GetTrackingState();
        output = pose_to_dict(tcw, tracking_state, timestamp, width, height, self->frame_index);
    } catch (const std::exception& exc) {
        PyErr_SetString(PyExc_RuntimeError, exc.what());
    }

    PyBuffer_Release(&view);
    return output;
}

PyObject* OrbSlam3Mono_shutdown(OrbSlam3MonoObject* self, PyObject*)
{
    if (self->slam) {
        self->slam->Shutdown();
        delete self->slam;
        self->slam = nullptr;
    }
    Py_RETURN_NONE;
}

PyMethodDef OrbSlam3Mono_methods[] = {
    {"track", reinterpret_cast<PyCFunction>(OrbSlam3Mono_track), METH_VARARGS, "Track a monocular uint8 numpy frame."},
    {"shutdown", reinterpret_cast<PyCFunction>(OrbSlam3Mono_shutdown), METH_NOARGS, "Shutdown ORB-SLAM3 threads."},
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject OrbSlam3MonoType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
};

PyModuleDef module_def = {
    PyModuleDef_HEAD_INIT,
    "orbslam3_py",
    "Minimal in-process ORB-SLAM3 Python module.",
    -1,
    nullptr,
};

}  // namespace

PyMODINIT_FUNC PyInit_orbslam3_py(void)
{
    OrbSlam3MonoType.tp_name = "orbslam3_py.OrbSlam3Mono";
    OrbSlam3MonoType.tp_basicsize = sizeof(OrbSlam3MonoObject);
    OrbSlam3MonoType.tp_itemsize = 0;
    OrbSlam3MonoType.tp_dealloc = reinterpret_cast<destructor>(OrbSlam3Mono_dealloc);
    OrbSlam3MonoType.tp_flags = Py_TPFLAGS_DEFAULT;
    OrbSlam3MonoType.tp_doc = "ORB-SLAM3 monocular tracker";
    OrbSlam3MonoType.tp_methods = OrbSlam3Mono_methods;
    OrbSlam3MonoType.tp_init = reinterpret_cast<initproc>(OrbSlam3Mono_init);
    OrbSlam3MonoType.tp_new = PyType_GenericNew;

    if (PyType_Ready(&OrbSlam3MonoType) < 0) {
        return nullptr;
    }

    PyObject* module = PyModule_Create(&module_def);
    if (!module) {
        return nullptr;
    }
    Py_INCREF(&OrbSlam3MonoType);
    if (PyModule_AddObject(module, "OrbSlam3Mono", reinterpret_cast<PyObject*>(&OrbSlam3MonoType)) < 0) {
        Py_DECREF(&OrbSlam3MonoType);
        Py_DECREF(module);
        return nullptr;
    }
    return module;
}
