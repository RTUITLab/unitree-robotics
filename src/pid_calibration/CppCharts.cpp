#include <Python.h>
#include <vector>

#include "CppCharts.hpp"

// Get mocked data for first line
std::vector<double> getData1()
{
	std::vector<double> line(1000);

	for (double i = 1; i <= 1000; i++)
	{
		line[(unsigned long)i] = (double)100 / i;
	}

	return line;
}

// Get mocked data for second line
std::vector<double> getData2()
{
	std::vector<double> line = {};

	line.resize(1000);

	for (double i = 1; i <= 1000; i++)
	{
		line[(unsigned long)i] = (double)100 / (i + 1.0);
	}

	return line;
}

void drawChart(std::vector< std::vector<double> > data, int leg)
{
	// Init Python
	Py_Initialize();

	// Setting current dir as a path to Python modules
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("import os");
	PyRun_SimpleString("sys.path.append(os.getcwd())");

	// Import module by its name. Module name is an object too
	PyObject *pName = PyUnicode_DecodeFSDefault((char*)"chart_example");
	PyObject *pModule = PyImport_Import(pName);

	Py_DECREF(pName);

	if (pModule != NULL)
	{
		// Get function from Python module: drawChart(data, leg)
		PyObject *pFunc = PyObject_GetAttrString(pModule, (char*)"drawChart");

		// Check that the function is correct
		if (pFunc && PyCallable_Check(pFunc))
		{
			// Convert vector of vectors to Python list of lists
			PyObject *pList = PyList_New(0);
			for (int i = 0; i < data.size(); i++)
			{
				PyObject *pLine = PyList_New(0);

				for (int j = 0; j < data[i].size(); j++)
				{
					PyObject *n = PyFloat_FromDouble(data[i][j]);
					PyList_Append(pLine, n);
				}

				PyList_Append(pList, pLine);
			}

			// Create function argument 
			PyObject *arg = PyTuple_New(2);
			PyTuple_SetItem(arg, 0, pList); // Setting first argument

			// Setting second argument 
			PyObject *pLeg = PyLong_FromLong(leg);
			PyTuple_SetItem(arg, 1, pLeg);

			// Execute function
			PyObject *pResult = PyObject_CallObject(pFunc, arg);
			PyErr_Print();
			printf("Finished\n");
		}
		else
		{
			printf("Wrong function\n");
		}
	}
	else
	{
		printf("Module is NULL\n");
	}
}

// Main function to tests

// int main(int argc, char* argv[])
// {
// 	std::vector< std::vector<double> > data = { getData1(), getData2() };
// 	drawChart(data);
// 	printf("%i\n", (int)data.size());
	
// 	return 0;
// }

