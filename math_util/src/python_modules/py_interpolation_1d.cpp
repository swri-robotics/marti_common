#include <Python.h>
#include <structmember.h>
#include <math_util/interpolation_1d.h>

#define CSTR(s) const_cast<char*>(s)

typedef struct {
  PyObject_HEAD
  math_util::Interpolation1D interp;
} PyInterpolation1D;


static PyMemberDef PyInterpolation1DMembers[] = {
  {NULL, 0, 0, 0, NULL} /* Sentinel */
};

static int PyInterpolation1D_init(PyInterpolation1D *self, PyObject *args, PyObject *kwds)
{
  return 0;
}

static void PyInterpolation1D_dealloc(PyInterpolation1D *self)
{
}

static PyObject* PyInterpolation1D_append_point(
  PyInterpolation1D *self,
  PyObject *args)
{
  double x, y;
  if (!PyArg_ParseTuple(args, "dd", &x, &y)) {
    return NULL;
  }

  bool success = self->interp.appendPoint(x,y);
  if (success) {
    Py_RETURN_TRUE;
  } else {
    Py_RETURN_FALSE;
  }
}

static PyObject* PyInterpolation1D_num_points(
  PyInterpolation1D *self,
  PyObject *args)
{
  return Py_BuildValue("n", self->interp.numPoints());
}

static PyObject* PyInterpolation1D_get_point(
  PyInterpolation1D *self,
  PyObject *args)
{
  size_t index;
  if (!PyArg_ParseTuple(args, "n", &index)) {
    return NULL;
  }
  
  std::pair<double, double> pt = self->interp.getPoint(index);  
  return Py_BuildValue("(dd)", pt.first, pt.second);
}

static PyObject* PyInterpolation1D_remove_point(
  PyInterpolation1D *self,
  PyObject *args)
{
  size_t index;
  if (!PyArg_ParseTuple(args, "n", &index)) {
    return NULL;
  }

  self->interp.removePoint(index);
  Py_RETURN_NONE;
}

static PyObject* PyInterpolation1D_clear(
  PyInterpolation1D *self,
  PyObject *args)
{
  self->interp.clear();
  Py_RETURN_NONE;
}

static PyObject* PyInterpolation1D_interpolation_type(
  PyInterpolation1D *self,
  PyObject *args)
{
  math_util::Interpolation1D::InterpolationType type =
    self->interp.interpolationType();

  std::string type_str = "<unknown>";
  if (type == math_util::Interpolation1D::ZERO_ORDER_HOLD) {
    type_str = "zero_order_hold";
  } else if (type == math_util::Interpolation1D::LINEAR) {
    type_str = "linear";
  }    
  
  return Py_BuildValue("s", type_str.c_str());
}

static PyObject* PyInterpolation1D_set_interpolation_type(
  PyInterpolation1D *self,
  PyObject *args)
{
  char *type_cstr;
  if (!PyArg_ParseTuple(args, "s", &type_cstr)) {
    return NULL;
  }

  std::string type_str(type_cstr);

  math_util::Interpolation1D::InterpolationType type;

  if (type_str == "zero_order_hold") {
    type = math_util::Interpolation1D::ZERO_ORDER_HOLD;
  } else if (type_str == "linear") {
    type = math_util::Interpolation1D::LINEAR;
  } else {
    return PyErr_Format(PyExc_ValueError,
                        "Invalid interpolation type (%s)",
                        type_cstr);
  }

  self->interp.setInterpolationType(type);
  Py_RETURN_NONE;
}

static PyObject* PyInterpolation1D_min_x(
  PyInterpolation1D *self,
  PyObject *args)
{
  return Py_BuildValue("d", self->interp.minX());
}

static PyObject* PyInterpolation1D_max_x(
  PyInterpolation1D *self,
  PyObject *args)
{
  return Py_BuildValue("d", self->interp.maxX());
}

static PyObject* PyInterpolation1D_eval(
  PyInterpolation1D *self,
  PyObject *args)
{
  double x;
  if (!PyArg_ParseTuple(args, "d", &x)) {
    return NULL;
  }

  double y = self->interp.eval(x);
  return Py_BuildValue("d", y);
}

static PyMethodDef PyInterpolation1DMethods[] = {
  {"append_point", (PyCFunction)PyInterpolation1D_append_point, METH_VARARGS, ""},
  {"num_points", (PyCFunction)PyInterpolation1D_num_points, METH_VARARGS, ""},
  {"get_point", (PyCFunction)PyInterpolation1D_get_point, METH_VARARGS, ""},
  {"remove_point", (PyCFunction)PyInterpolation1D_remove_point, METH_VARARGS, ""},
  {"clear", (PyCFunction)PyInterpolation1D_clear, METH_VARARGS, ""},
  {"interpolation_type", (PyCFunction)PyInterpolation1D_interpolation_type, METH_VARARGS, ""},
  {"set_interpolation_type", (PyCFunction)PyInterpolation1D_set_interpolation_type, METH_VARARGS, ""},
  {"min_x", (PyCFunction)PyInterpolation1D_min_x, METH_VARARGS, ""},
  {"max_x", (PyCFunction)PyInterpolation1D_max_x, METH_VARARGS, ""},
  {"eval", (PyCFunction)PyInterpolation1D_eval, METH_VARARGS, ""},
  {NULL, NULL, 0, NULL}  /* Sentinel */
};


PyTypeObject PyInterpolation1DType = {
    PyObject_HEAD_INIT(NULL)
    0,                                  /*ob_size*/
    "_pwm.PyInterpolation1D",  /*tp_name*/
    sizeof(PyInterpolation1D),   /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    (destructor)PyInterpolation1D_dealloc, /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_compare*/
    0,                         /*tp_repr*/
    0,                         /*tp_as_number*/
    0,                         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
    "PyInterpolation1D object",  /* tp_doc */
    0,		               /* tp_traverse */
    0,		               /* tp_clear */
    0,		               /* tp_richcompare */
    0,		               /* tp_weaklistoffset */
    0,		               /* tp_iter */
    0,		               /* tp_iternext */
    PyInterpolation1DMethods,    /* tp_methods */
    PyInterpolation1DMembers,    /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)PyInterpolation1D_init,  /* tp_init */
    0,                         /* tp_alloc */
    0,                        /* tp_new */
    0, /* tp_free */
    0, /* tp_is_gc */
    0, /* tp_bases */
    0, /* tp_mro */
    0, /* tp_cache */
    0, /* tp_subclasses */
    0, /* tp_weaklist */
    0, /* tp_del */
    0  /* tp_version_tag */
};
