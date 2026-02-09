/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 *	   $Id$
 *
 *****************************************************************************/

#include "bindings.h"
#include "DataFieldDescIteratorBinder.h"
#include "Root.h"
#include "RootBinder.h"
#include "ServiceBinder.h"
#include "StringIteratorBinder.h"
#include "logger.h"

namespace Darkness {
Darkness::Root *Darkness::PythonLanguage::msRoot = NULL;

namespace Python {

PyObject *VariantToPyObject(const Variant &inst) {
    // Do a conversion to python object from variant
    // Look for the type
    switch (inst.type()) {
    case Variant::DV_INVALID:
        Py_INCREF(Py_None);
        return Py_None;

    case Variant::DV_BOOL:
        return TypeInfo<bool>::toPyObject(inst.toBool());
    case Variant::DV_FLOAT:
        return TypeInfo<float>::toPyObject(inst.toFloat());
    case Variant::DV_INT:
        return TypeInfo<int>::toPyObject(inst.toInt());
    case Variant::DV_UINT:
        return TypeInfo<int>::toPyObject(inst.toUInt());
    case Variant::DV_STRING:
        return TypeInfo<std::string>::toPyObject(inst.toString());
    case Variant::DV_VECTOR:
        return TypeInfo<Vector3>::toPyObject(inst.toVector());
    case Variant::DV_QUATERNION:
        return TypeInfo<Quaternion>::toPyObject(inst.toQuaternion());
    default: // All possible paths must return a value
        PyErr_SetString(PyExc_TypeError, "Invalid Variant type");
        return NULL;
    }
}

Variant PyObjectToVariant(PyObject *obj) {
    // Do a conversion from python object to Variant instance
    // Look for the type of the python object

    if (PyLong_Check(obj))
        return Variant(static_cast<int>(PyLong_AsLong(obj)));
    else if (PyBool_Check(obj)) {
        if (obj == Py_True)
            return Variant((bool)true);
        else
            return Variant((bool)false);
    } else if (PyFloat_Check(obj))
        return Variant((float)PyFloat_AsDouble(obj));
#ifdef IS_PY3K
    else if (PyBytes_Check(obj))
        return Variant(PyBytes_AsString(obj));
#else
    else if (PyString_Check(obj))
        return Variant(PyString_AsString(obj));
#endif
    else if (PyModule_Check(obj)) {
        float x, y, z, w;

        if (PyArg_Parse(obj, "[ffff]", &x, &y, &z, &w)) {
            return Variant(x, y, z, w);
        } else if (PyArg_Parse(obj, "[fff]", &x, &y, &z)) {
            return Variant(x, y, z);
        } else
            return Variant(Variant::DV_INVALID);
    }

    return Variant(Variant::DV_INVALID); // Py_None, or a non-handled type
}

// Logging methods
// LEVELS: fatal error info debug verbose
PyObject *Py_Log(PyObject *self, Logger::LogLevel level, PyObject *args) {
    const char *text;
    PyObject *result = NULL;

    if (PyArg_ParseTuple(args, "s", &text)) {
        Logger::getSingleton().log(level, "Python: %s", text);

        result = Py_None;
        Py_INCREF(result);
    } else {
        PyErr_SetString(PyExc_TypeError, "Expected string argument!");
    }

    return result;
}

PyObject *Py_Log_Fatal(PyObject *self, PyObject *args) {
    return Py_Log(self, Logger::LOG_LEVEL_FATAL, args);
}

PyObject *Py_Log_Error(PyObject *self, PyObject *args) {
    return Py_Log(self, Logger::LOG_LEVEL_ERROR, args);
}

PyObject *Py_Log_Info(PyObject *self, PyObject *args) {
    return Py_Log(self, Logger::LOG_LEVEL_INFO, args);
}

PyObject *Py_Log_Debug(PyObject *self, PyObject *args) {
    return Py_Log(self, Logger::LOG_LEVEL_DEBUG, args);
}

PyObject *Py_Log_Verbose(PyObject *self, PyObject *args) {
    return Py_Log(self, Logger::LOG_LEVEL_VERBOSE, args);
}

// ------------------------------------------------------------------------------------------------
void PythonPublishedType::publishType(PyObject *containter, PyTypeObject *type,
                                      const char *name) {
    PyType_Ready(type);
    Py_INCREF(type);

    // VC does not like const to plain C calls. So we have to const_cast here
    PyModule_AddObject(containter, const_cast<char *>(name), (PyObject *)type);
}
} // namespace Python

// ---- Doc Strings ----
const char *darkness_log_fatal__doc__ = "log_fatal(msg)\n"
                                    "\tLogs a message with FATAL log level.\n"
                                    "@type msg: string\n"
                                    "@param msg: The logged string\n";

const char *darkness_log_error__doc__ = "log_error(msg)\n"
                                    "\tLogs a message with ERROR log level.\n"
                                    "@type msg: string\n"
                                    "@param msg: The logged string\n";
;

const char *darkness_log_info__doc__ = "log_info(msg)\n"
                                   "\tLogs a message with INFO log level.\n"
                                   "@type msg: string\n"
                                   "@param msg: The logged string\n";

const char *darkness_log_debug__doc__ = "log_debug(msg)\n"
                                    "\tLogs a message with DEBUG log level.\n"
                                    "@type msg: string\n"
                                    "@param msg: The logged string\n";

const char *darkness_log_verbose__doc__ =
    "log_verbose(msg)\n"
    "\tLogs a message with VERBOSE (=ALL) log level.\n"
    "@type msg: string\n"
    "@param msg: The logged string\n";

const char *darkness_createRoot__doc__ =
    "createRoot(mask;logfile)\n"
    "Creates the Darkness.Root object with the specified service mask (See "
    "L{darkness.services<Darkness.Services>}).\n"
    "@type mask: number\n"
    "@param mask: Service creation mask\n"
    "@type logfile: string\n"
    "@param logfile: The log file (optional)\n"
    "@rtype: Root\n"
    "@return: A new darkness.Root object reference";

const char *darkness_getRoot__doc__ =
    "getRoot()\n"
    "Retrieves the previously created darkness.Root object.\n"
    "@rtype: Root\n"
    "@return: An darkness.Root object reference";

PyMethodDef sDarknessMethods[] = {
    {const_cast<char *>("log_fatal"), Python::Py_Log_Fatal, METH_VARARGS,
     darkness_log_fatal__doc__},
    {const_cast<char *>("log_error"), Python::Py_Log_Error, METH_VARARGS,
     darkness_log_error__doc__},
    {const_cast<char *>("log_info"), Python::Py_Log_Info, METH_VARARGS,
     darkness_log_info__doc__},
    {const_cast<char *>("log_debug"), Python::Py_Log_Debug, METH_VARARGS,
     darkness_log_debug__doc__},
    {const_cast<char *>("log_verbose"), Python::Py_Log_Verbose, METH_VARARGS,
     darkness_log_verbose__doc__},
    {const_cast<char *>("createRoot"), PythonLanguage::createRoot, METH_VARARGS,
     darkness_createRoot__doc__},
    {const_cast<char *>("getRoot"), PythonLanguage::getRoot, METH_NOARGS,
     darkness_getRoot__doc__},
    {NULL, NULL},
};

#ifdef IS_PY3K
// this is tricky. For python 3 we need some more boilerplate

static int darknessmodule_traverse(PyObject *m, visitproc visit, void *arg) {
    Py_VISIT(GETSTATE(m)->error);
    return 0;
}

static int darknessmodule_clear(PyObject *m) {
    Py_CLEAR(GETSTATE(m)->error);
    return 0;
}

static struct PyModuleDef sDarknessModuleDef = {
    PyModuleDef_HEAD_INIT,       "darkness",           NULL,
    sizeof(struct module_state), sDarknessMethods,     NULL,
    darknessmodule_traverse,         darknessmodule_clear, NULL};
#else
static struct module_state _state;
#endif

void PythonLanguage::init(int argc, char **argv) {
#ifdef IS_PY3K
    // insert our module into the init tab
    PyImport_AppendInittab("darkness", &initModule);
#endif
    Py_Initialize();

#ifndef IS_PY3K
    initModule();
#endif

    if (PyErr_Occurred()) {
        // TODO: Do something useful here, or forget it
        PyErr_Print();
        PyErr_Clear();
    }

#warning Fix this for python3!
#ifndef IS_PY3K
    PySys_SetArgv(argc, argv);
#endif
}

PyObject *PythonLanguage::initModule() {
    msRoot = NULL;

    // Create an Darkness module
#ifdef IS_PY3K
    PyObject *module = PyModule_Create(&sDarknessModuleDef);
#else
    PyObject *module = Py_InitModule("darkness", sDarknessMethods);
#endif
    // Error?
    if (!module)
        DARKNESS_EXCEPT("Could not initialize the python Module!");

    // Error handling
    struct module_state *st = GETSTATE(module);
    st->error = PyErr_NewException("darkness.Error", NULL, NULL);
    if (st->error == NULL) {
        Py_DECREF(module);
        DARKNESS_EXCEPT("Could not initialize the darkness.Error!");
    }

    // Call all the binders here. The result is initialized Python VM
    // PyObject *servicemod =
    Python::ServiceBinder::init(module);
    Python::RootBinder::init(module);

    Python::StringIteratorBinder::init(module);
    Python::DataFieldsBinder::init(module);
    return module;
}

void PythonLanguage::term() {
    Py_Finalize();

    // If we constructed the root, destroy it now...
    if (msRoot)
        delete msRoot;
}

void PythonLanguage::runScriptPtr(const char *ptr) {
    // Is this the right way?
    PyRun_SimpleString(ptr);

    if (PyErr_Occurred()) {
        // TODO: Do something useful here, or forget it
        // TODO: PythonException(BasicException) with the PyErr string probably.
        // Same in the init
        PyErr_Print();
        PyErr_Clear();
    }
}

bool PythonLanguage::runScript(const char *fname) {
    FILE *fp = fopen(fname, "rb");

    if (fp != NULL) {
        /* A short explanation:
           1. The PyRun_SimpleFile has issues with FILE* type. Results in Access
           Violations on Windows
           2. The PyRun_SimpleString does not handle DOS line-endings.
        */

        std::ifstream pyfile(fname);

        if (!pyfile)
            return false;

        std::string ftxt, line;

        while (!pyfile.eof()) {
            getline(pyfile, line);
            ftxt += line;
            ftxt += '\n';
        }
        pyfile.close();

        PyRun_SimpleStringFlags(ftxt.c_str(), NULL);
    }

    __PY_HANDLE_PYTHON_ERROR;

    return true;
}

PyObject *PythonLanguage::createRoot(PyObject *self, PyObject *args) {
    // args: Module mask - unsigned long long
    PyObject *result = NULL;
    const char *logfile = NULL;
    unsigned long mask;

    if (PyArg_ParseTuple(args, "l|s", &mask, &logfile)) {
        if (logfile) {
            // Create new root, wrap, return
            msRoot = new Darkness::Root(mask, logfile);
        } else {
            // new root without logfile specifier
            msRoot = new Darkness::Root(mask);
        }

        // Todo: We could also share a single object instance here PyObject -
        // mPyRoot PyIncRef on it, return
        result = Python::RootBinder::create(msRoot);
        return result;
    } else {
        // Invalid parameters
        PyErr_SetString(PyExc_TypeError, "Expected one integer argument!");
        return NULL;
    }
}

PyObject *PythonLanguage::getRoot(PyObject *self, PyObject *args) {
    // args: Module mask - unsigned long long
    PyObject *result = NULL;

    if (msRoot == NULL) {
        // Invalid parameters
        // TODO: Choose a propper exception type
        PyErr_SetString(PyExc_TypeError, "Root was not yet created!");
        return NULL;
    }

    result = Python::RootBinder::create(msRoot);
    return result;
}

} // namespace Darkness
