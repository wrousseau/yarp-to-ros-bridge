/*
 * Copyright (C) 2013 MACSi Project
 * Author: Woody Rousseau
 * email:  woody.rousseau@ensta-paristech.fr
 * website: www.macsi.isir.upmc.fr
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \file imageThread.h
 * \brief ImageThread class
 * \author Woody Rousseau
 * \version 0.1
 * \date 11/06/13
 *
 * Using Python in C++
 */

#ifndef PYTHON_WRAPPER_H
#define PYTHON_WRAPPER_H

#include <Python.h>

#include <exception>
#include <iterator>
#include <string>
#include <vector>
#include <iostream>

using namespace std;

/** \class Python_exception
   * \brief Basic Python exception class inheriting from std::exception
   */
class Python_exception : public std::exception
{
public:
	Python_exception(const std::string& m);
    virtual ~Python_exception() throw();
    virtual const char* what() throw();
private:
	std::string message;
};

/**
 * \brief Wraps a Python script
 *
 * \param scriptName the name of the script which will be interpreted
 * \param functionName the name of the entry function which will be ran when the scripts begins
 * \param arguments a vector containing the arguments to pass to functionName
 * \return A string which must be the output of the Python script
 *
 * The Python.h header is heavily used and does weird things memory-wise
 * This makes studies with tools such as Valgrind almost impossible
 */
string pythonWrap(string scriptName, string functionName, vector<string> arguments);

#endif