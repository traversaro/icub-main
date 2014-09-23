/*
 * Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "Debug.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <stdlib.h>
#include <string.h>

#include <yarp/os/Os.h>


std::ofstream DebugStream::Debug::ftrc;
std::ofstream DebugStream::Debug::fout;
std::ofstream DebugStream::Debug::ferr;

#ifndef WIN32

 #define RED    (colored_output ? "\033[01;31m" : "")
 #define GREEN  (colored_output ? "\033[01;32m" : "")
 #define YELLOW (colored_output ? "\033[01;33m" : "")
 #define BLUE   (colored_output ? "\033[01;34m" : "")
 #define CLEAR  (colored_output ? "\033[00m" : "")

 bool DebugStream::Debug::colored_output(getenv("ICUB_COLORED_OUTPUT") && (strcmp(getenv("ICUB_COLORED_OUTPUT"), "1") == 0));
 bool DebugStream::Debug::verbose_output(getenv("ICUB_VERBOSE_OUTPUT") && (strcmp(getenv("ICUB_VERBOSE_OUTPUT"), "1") == 0));
 bool DebugStream::Debug::trace_output(getenv("ICUB_TRACE_ENABLE") && (strcmp(getenv("ICUB_TRACE_ENABLE"), "1") == 0));

#else // WIN32

 // TODO colored and verbose_output for WIN32
 #define RED    ""
 #define GREEN  ""
 #define YELLOW ""
 #define BLUE   ""
 #define CLEAR  ""

 bool DebugStream::Debug::colored_output(false);
 bool DebugStream::Debug::verbose_output(false);
 bool DebugStream::Debug::trace_output(false);
#endif // WIN32


void DebugStream::Debug::print_output(MsgType t,
                                         const std::ostringstream &s,
                                         const char *file,
                                         unsigned int line,
                                         const char *func)
{
    switch (t) {
    case TraceType:
        if(trace_output)
        if (ftrc.is_open()) {
            if (verbose_output) {
                ftrc << "T: " << file << ":" << line << " " << func << ":" << s.str() << std::endl;
            } else {
                ftrc << "TRACE: " << func << s.str() << std::endl;
            }
        } else {
            if (verbose_output) {
                std::cout << GREEN << "T" << CLEAR << ": " << file << ":" << line << " " << GREEN << func << CLEAR << ": " << s.str() << std::endl;
            } else {
                std::cout << GREEN << "TRACE" << CLEAR << ": " << func << s.str() << std::endl;
            }
        }
        break;
    case DebugType:
        if (fout.is_open()) {
            if (verbose_output) {
            	fout << "D: " << file << ":" << line << " " << func << ":" << s.str() << std::endl;
            } else {
                fout << "DEBUG: " << s.str() << std::endl;
            }
        } else {
            if (verbose_output) {
                std::cout << BLUE << "D" << CLEAR << ": " << file << ":" << line << " " << BLUE << func << CLEAR << ": " << s.str() << std::endl;
            } else {
                std::cout << "[" << BLUE << "DEBUG" << CLEAR << "]" << s.str() << std::endl;
            }
        }
        break;

    case WarningType:
        if (ferr.is_open()) {
            if (verbose_output) {
                ferr << "W: " << file << ":" << line << " " << func << ":" << s.str() << std::endl;
            } else {
                ferr << "WARNING: " << s.str() << std::endl;
            }
        }
        if (verbose_output) {
            std::cerr << YELLOW << "W" << CLEAR << ": " << file << ":" << line << " " << YELLOW << func << CLEAR << ": " << s.str() << std::endl;
        } else {
            std::cerr << "[" << YELLOW << "WARNING" << CLEAR << "]" << s.str() << std::endl;
        }
        break;
    case ErrorType:
        if (ferr.is_open()) {
            if (verbose_output) {
                ferr << "E: " << file << ":" << line << " " << func << ":" << s.str() << std::endl;
            } else {
                ferr << "ERROR: " << s.str() << std::endl;
            }
        }
        if (verbose_output) {
            std::cerr << RED << "E" << CLEAR << ": " << file << ":" << line << " " << RED << func << CLEAR << ": " << s.str() << std::endl;
        } else {
            std::cerr << "[" << RED << "ERROR" << CLEAR << "]" << s.str() << std::endl;
        }
        break;
    case FatalType:
        if (ferr.is_open()) {
            if (verbose_output) {
                ferr << "F: " << file << ":" << line << " " << func << ":" << s.str() << std::endl;
            } else {
                ferr << "FATAL: " << s.str() << std::endl;
            }
        }
        if (verbose_output) {
            std::cerr << RED << "F" << CLEAR << ": " << file << ":" << line << " " << RED << func << CLEAR << ": " << s.str() << std::endl;
        } else {
            std::cerr << RED << "FATAL" << CLEAR << ": " << s.str() << std::endl;
        }
        yarp::os::exit(-1);
        break;
    default:
        break;
    }
}

void DebugStream::Debug::setTraceFile(const std::string& filename)
{
    if(ftrc.is_open()) {
        ftrc.close();
    }
    ftrc.open(filename.c_str());
}


void DebugStream::Debug::setOutputFile(const std::string& filename)
{
    if(fout.is_open()) {
        fout.close();
    }
    fout.open(filename.c_str());
}

void DebugStream::Debug::setErrorFile(const std::string& filename)
{
    if(ferr.is_open()) {
        ferr.close();
    }
    ferr.open(filename.c_str());
}
