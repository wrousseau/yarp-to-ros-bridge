#include "pythonWrapper.h"

string pythonWrap(string scriptName, string functionName, vector<string> arguments)
{
    PyObject *pName, *pModule, *pFunc;
    PyObject *pValue, *pArgs;

    Py_Initialize();
    pName = PyString_FromString(scriptName.c_str());
    PyRun_SimpleString("import sys"); 
    PyRun_SimpleString("sys.path.append(\"./python\")");
    PyRun_SimpleString("sys.argv=['']");
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) 
    {
        pFunc = PyObject_GetAttrString(pModule, functionName.c_str());
        if (pFunc && PyCallable_Check(pFunc)) 
        {
            pArgs = PyTuple_New(arguments.size());
            for (auto itt = arguments.begin(); itt != arguments.end(); ++itt)
            {
                pValue = PyString_FromString(itt->c_str());
                PyTuple_SetItem(pArgs, itt - arguments.begin(), pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            if (pValue == NULL)
            {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                throw Python_exception("The Python script "+scriptName+" did not return");
            }
        }
        else 
        {
            throw Python_exception("The Python script " + scriptName + ".py was somehow modified. Please redownload.");  
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else 
    {
        throw Python_exception("The Python script " + scriptName + ".py does not seem to exist. Please redownload.");
    }
    string result = PyString_AsString(pValue);
    Py_DECREF(pValue);
    Py_Finalize();

    return result;
}

Python_exception::Python_exception(const std::string& m)
{
    message = m;
}

Python_exception::~Python_exception() throw()
{
}

const char* 
Python_exception::what() throw()
{
  return message.c_str();
}