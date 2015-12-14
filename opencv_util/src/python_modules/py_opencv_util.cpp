#include <Python.h>
#include <opencv2/core/core.hpp>
#include <opencv_util/model_fit.h>

#define PY_ARRAY_UNIQUE_SYMBOL cvutil_ARRAY_API
#include "numpy/arrayobject.h"

static bool validArray(const std::string &name,
                       PyArrayObject *array)
{
  if (PyArray_NDIM(array) != 2) {
    PyErr_Format(PyExc_TypeError, "%s has %d dimensions instead of 2.",
                 name.c_str(),
                 PyArray_NDIM(array));
    return false;
  }

  if (PyArray_DIM(array, 1) != 2) {
    return PyErr_Format(PyExc_TypeError, "%s has %d columns instead of 2.",
                        name.c_str(),
                        PyArray_NDIM(array));
    return false;
  }

  if (PyArray_TYPE(array) != NPY_FLOAT32) {
    PyErr_Format(PyExc_TypeError, "%s is not a np.float32 typed array.", name.c_str());
    return false;
  }
  
  return true;
}

static PyObject* fitRigidTransform2d(
  PyObject *self,
  PyObject *args)
{
  PyArrayObject *points1_array;
  PyArrayObject *points2_array;

  bool valid_args = PyArg_ParseTuple(args, "O!O!",
                                     &PyArray_Type, &points1_array,
                                     &PyArray_Type, &points2_array);

  if (!valid_args) {
    return NULL;
  }

  if (!validArray("points1", points1_array)) {
    return NULL;
  }
  if (!validArray("points2", points2_array)) {
    return NULL;
  }

  if (PyArray_DIM(points1_array, 0) != PyArray_DIM(points2_array, 0)) {
    return PyErr_Format(PyExc_TypeError, "points1 and points2 do not have the "
                        "same number of rows (%zd vs %zd).",
                        PyArray_DIM(points1_array, 0),
                        PyArray_DIM(points2_array, 0));
  }

  ssize_t n = PyArray_DIM(points1_array, 0);
  cv::Mat points1_mat(n, 1, CV_32FC2);
  cv::Mat points2_mat(n, 1, CV_32FC2);

  float* points1_src = reinterpret_cast<float*>(PyArray_DATA(points1_array));
  float* points2_src = reinterpret_cast<float*>(PyArray_DATA(points2_array));
  float* points1_dst = points1_mat.ptr<float>(0);
  float* points2_dst = points2_mat.ptr<float>(0);

  for (ssize_t i = 0; i < n*2; i++) {
    points1_dst[i] = points1_src[i];
    points2_dst[i] = points2_src[i];
  }

  cv::Mat transform_mat = opencv_util::FitRigidTransform2d(points1_mat, points2_mat);

  if (transform_mat.empty()) {
    Py_RETURN_NONE;
  }

  if (transform_mat.rows != 2 || transform_mat.cols != 3) {
    return PyErr_Format(PyExc_TypeError, "FitRigidTranasform2d return unexpected size (%d x %d)",
                        transform_mat.rows, transform_mat.cols);
  }

  npy_intp dims[2];
  dims[0] = 2;
  dims[1] = 3;
  PyObject *transform_array = PyArray_SimpleNew(2, dims, NPY_FLOAT32);
  float *transform_dst = reinterpret_cast<float*>(PyArray_DATA(transform_array));
  float *transform_src = transform_mat.ptr<float>(0);

  for (size_t i = 0; i < 6; i++) {
    transform_dst[i] = transform_src[i];
  }

  return transform_array; 
}

static PyMethodDef module_methods[] = {
  {"_fit_rigid_transform_2d", (PyCFunction)fitRigidTransform2d, METH_VARARGS, ""},
    {NULL}  /* Sentinel */
};

PyMODINIT_FUNC
init_opencv_util(void)
{
  PyObject *m;
  
  m = Py_InitModule("_opencv_util", module_methods);

  import_array();
}
