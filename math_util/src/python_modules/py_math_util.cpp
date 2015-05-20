#include <Python.h>

extern PyTypeObject PyInterpolation1DType;

static PyMethodDef MathUtilMethods[] = {
  {NULL, NULL, 0, NULL}  /* Sentinel */
};

PyMODINIT_FUNC
init_math_util(void)
{
  PyObject *m;
  
  PyInterpolation1DType.tp_new = PyType_GenericNew;
  if (PyType_Ready(&PyInterpolation1DType) < 0)
    return;
  
  m = Py_InitModule("_math_util", MathUtilMethods);

  Py_INCREF(reinterpret_cast<PyTypeObject*>(&PyInterpolation1DType));
  PyModule_AddObject(m, "PyInterpolation1D", (PyObject *)&PyInterpolation1DType);
}
