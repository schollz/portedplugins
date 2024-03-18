/* ------------------------------------------------------------
author: "Jean Pierre Cimalando"
license: "BSD-2-Clause"
name: "fverb2"
version: "0.5"
Code generated with Faust 2.70.3 (https://faust.grame.fr)
Compilation options: -a supercollider.cpp -lang cpp -i -ct 1 -es 1 -mcd 16 -mdd
1024 -mdy 33 -single -ftz 0
------------------------------------------------------------ */

#ifndef __mydsp_H__
#define __mydsp_H__

/************************************************************************
 IMPORTANT NOTE : this file contains two clearly delimited sections :
 the ARCHITECTURE section (in two parts) and the USER section. Each section
 is governed by its own copyright and license. Please check individually
 each section for license and copyright information.
 *************************************************************************/

/******************* BEGIN supercollider.cpp ****************/
/************************************************************************
 FAUST Architecture File
 Copyright (C) 2005-2012 Stefan Kersten.
 Copyright (C) 2003-2019 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This Architecture section is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 3 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; If not, see <http://www.gnu.org/licenses/>.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.

 ************************************************************************
 ************************************************************************/

// The prefix is set to "Faust" in the faust2supercollider script, otherwise set
// empty
#if !defined(SC_FAUST_PREFIX)
#define SC_FAUST_PREFIX ""
#endif

#include <SC_PlugIn.h>
#include <string.h>

#include <map>
#include <string>

/************************** BEGIN dsp.h ********************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

#ifndef __dsp__
#define __dsp__

#include <cstdint>
#include <string>
#include <vector>

/************************************************************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ***************************************************************************/

#ifndef __export__
#define __export__

// Version as a global string
#define FAUSTVERSION "2.70.3"

// Version as separated [major,minor,patch] values
#define FAUSTMAJORVERSION 2
#define FAUSTMINORVERSION 70
#define FAUSTPATCHVERSION 3

// Use FAUST_API for code that is part of the external API but is also compiled
// in faust and libfaust Use LIBFAUST_API for code that is compiled in faust and
// libfaust

#ifdef _WIN32
#pragma warning(disable : 4251)
#ifdef FAUST_EXE
#define FAUST_API
#define LIBFAUST_API
#elif FAUST_LIB
#define FAUST_API __declspec(dllexport)
#define LIBFAUST_API __declspec(dllexport)
#else
#define FAUST_API
#define LIBFAUST_API
#endif
#else
#ifdef FAUST_EXE
#define FAUST_API
#define LIBFAUST_API
#else
#define FAUST_API __attribute__((visibility("default")))
#define LIBFAUST_API __attribute__((visibility("default")))
#endif
#endif

#endif

#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif

struct FAUST_API UI;
struct FAUST_API Meta;

/**
 * DSP memory manager.
 */

struct FAUST_API dsp_memory_manager {
  virtual ~dsp_memory_manager() {}

  /**
   * Inform the Memory Manager with the number of expected memory zones.
   * @param count - the number of expected memory zones
   */
  virtual void begin(size_t /*count*/) {}

  /**
   * Give the Memory Manager information on a given memory zone.
   * @param size - the size in bytes of the memory zone
   * @param reads - the number of Read access to the zone used to compute one
   * frame
   * @param writes - the number of Write access to the zone used to compute one
   * frame
   */
  virtual void info(size_t /*size*/, size_t /*reads*/, size_t /*writes*/) {}

  /**
   * Inform the Memory Manager that all memory zones have been described,
   * to possibly start a 'compute the best allocation strategy' step.
   */
  virtual void end() {}

  /**
   * Allocate a memory zone.
   * @param size - the memory zone size in bytes
   */
  virtual void* allocate(size_t size) = 0;

  /**
   * Destroy a memory zone.
   * @param ptr - the memory zone pointer to be deallocated
   */
  virtual void destroy(void* ptr) = 0;
};

/**
 * Signal processor definition.
 */

class FAUST_API dsp {
 public:
  dsp() {}
  virtual ~dsp() {}

  /* Return instance number of audio inputs */
  virtual int getNumInputs() = 0;

  /* Return instance number of audio outputs */
  virtual int getNumOutputs() = 0;

  /**
   * Trigger the ui_interface parameter with instance specific calls
   * to 'openTabBox', 'addButton', 'addVerticalSlider'... in order to build the
   * UI.
   *
   * @param ui_interface - the user interface builder
   */
  virtual void buildUserInterface(UI* ui_interface) = 0;

  /* Return the sample rate currently used by the instance */
  virtual int getSampleRate() = 0;

  /**
   * Global init, calls the following methods:
   * - static class 'classInit': static tables initialization
   * - 'instanceInit': constants and instance state initialization
   *
   * @param sample_rate - the sampling rate in Hz
   */
  virtual void init(int sample_rate) = 0;

  /**
   * Init instance state
   *
   * @param sample_rate - the sampling rate in Hz
   */
  virtual void instanceInit(int sample_rate) = 0;

  /**
   * Init instance constant state
   *
   * @param sample_rate - the sampling rate in Hz
   */
  virtual void instanceConstants(int sample_rate) = 0;

  /* Init default control parameters values */
  virtual void instanceResetUserInterface() = 0;

  /* Init instance state (like delay lines...) but keep the control parameter
   * values */
  virtual void instanceClear() = 0;

  /**
   * Return a clone of the instance.
   *
   * @return a copy of the instance on success, otherwise a null pointer.
   */
  virtual dsp* clone() = 0;

  /**
   * Trigger the Meta* parameter with instance specific calls to 'declare' (key,
   * value) metadata.
   *
   * @param m - the Meta* meta user
   */
  virtual void metadata(Meta* m) = 0;

  /**
   * DSP instance computation, to be called with successive in/out audio
   * buffers.
   *
   * @param count - the number of frames to compute
   * @param inputs - the input audio buffers as an array of non-interleaved
   * FAUSTFLOAT samples (eiher float, double or quad)
   * @param outputs - the output audio buffers as an array of non-interleaved
   * FAUSTFLOAT samples (eiher float, double or quad)
   *
   */
  virtual void compute(int count, FAUSTFLOAT** inputs,
                       FAUSTFLOAT** outputs) = 0;

  /**
   * DSP instance computation: alternative method to be used by subclasses.
   *
   * @param date_usec - the timestamp in microsec given by audio driver.
   * @param count - the number of frames to compute
   * @param inputs - the input audio buffers as an array of non-interleaved
   * FAUSTFLOAT samples (either float, double or quad)
   * @param outputs - the output audio buffers as an array of non-interleaved
   * FAUSTFLOAT samples (either float, double or quad)
   *
   */
  virtual void compute(double /*date_usec*/, int count, FAUSTFLOAT** inputs,
                       FAUSTFLOAT** outputs) {
    compute(count, inputs, outputs);
  }
};

/**
 * Generic DSP decorator.
 */

class FAUST_API decorator_dsp : public dsp {
 protected:
  dsp* fDSP;

 public:
  decorator_dsp(dsp* dsp = nullptr) : fDSP(dsp) {}
  virtual ~decorator_dsp() { delete fDSP; }

  virtual int getNumInputs() { return fDSP->getNumInputs(); }
  virtual int getNumOutputs() { return fDSP->getNumOutputs(); }
  virtual void buildUserInterface(UI* ui_interface) {
    fDSP->buildUserInterface(ui_interface);
  }
  virtual int getSampleRate() { return fDSP->getSampleRate(); }
  virtual void init(int sample_rate) { fDSP->init(sample_rate); }
  virtual void instanceInit(int sample_rate) {
    fDSP->instanceInit(sample_rate);
  }
  virtual void instanceConstants(int sample_rate) {
    fDSP->instanceConstants(sample_rate);
  }
  virtual void instanceResetUserInterface() {
    fDSP->instanceResetUserInterface();
  }
  virtual void instanceClear() { fDSP->instanceClear(); }
  virtual decorator_dsp* clone() { return new decorator_dsp(fDSP->clone()); }
  virtual void metadata(Meta* m) { fDSP->metadata(m); }
  // Beware: subclasses usually have to overload the two 'compute' methods
  virtual void compute(int count, FAUSTFLOAT** inputs, FAUSTFLOAT** outputs) {
    fDSP->compute(count, inputs, outputs);
  }
  virtual void compute(double date_usec, int count, FAUSTFLOAT** inputs,
                       FAUSTFLOAT** outputs) {
    fDSP->compute(date_usec, count, inputs, outputs);
  }
};

/**
 * DSP factory class, used with LLVM and Interpreter backends
 * to create DSP instances from a compiled DSP program.
 */

class FAUST_API dsp_factory {
 protected:
  // So that to force sub-classes to use deleteDSPFactory(dsp_factory* factory);
  virtual ~dsp_factory() {}

 public:
  /* Return factory name */
  virtual std::string getName() = 0;

  /* Return factory SHA key */
  virtual std::string getSHAKey() = 0;

  /* Return factory expanded DSP code */
  virtual std::string getDSPCode() = 0;

  /* Return factory compile options */
  virtual std::string getCompileOptions() = 0;

  /* Get the Faust DSP factory list of library dependancies */
  virtual std::vector<std::string> getLibraryList() = 0;

  /* Get the list of all used includes */
  virtual std::vector<std::string> getIncludePathnames() = 0;

  /* Get warning messages list for a given compilation */
  virtual std::vector<std::string> getWarningMessages() = 0;

  /* Create a new DSP instance, to be deleted with C++ 'delete' */
  virtual dsp* createDSPInstance() = 0;

  /* Static tables initialization, possibly implemened in sub-classes*/
  virtual void classInit(int sample_rate){};

  /* Set a custom memory manager to be used when creating instances */
  virtual void setMemoryManager(dsp_memory_manager* manager) = 0;

  /* Return the currently set custom memory manager */
  virtual dsp_memory_manager* getMemoryManager() = 0;
};

// Denormal handling

#if defined(__SSE__)
#include <xmmintrin.h>
#endif

class FAUST_API ScopedNoDenormals {
 private:
  intptr_t fpsr = 0;

  void setFpStatusRegister(intptr_t fpsr_aux) noexcept {
#if defined(__arm64__) || defined(__aarch64__)
    asm volatile("msr fpcr, %0" : : "ri"(fpsr_aux));
#elif defined(__SSE__)
    // The volatile keyword here is needed to workaround a bug in
    // AppleClang 13.0 which aggressively optimises away the variable otherwise
    volatile uint32_t fpsr_w = static_cast<uint32_t>(fpsr_aux);
    _mm_setcsr(fpsr_w);
#endif
  }

  void getFpStatusRegister() noexcept {
#if defined(__arm64__) || defined(__aarch64__)
    asm volatile("mrs %0, fpcr" : "=r"(fpsr));
#elif defined(__SSE__)
    fpsr = static_cast<intptr_t>(_mm_getcsr());
#endif
  }

 public:
  ScopedNoDenormals() noexcept {
#if defined(__arm64__) || defined(__aarch64__)
    intptr_t mask = (1 << 24 /* FZ */);
#elif defined(__SSE__)
#if defined(__SSE2__)
    intptr_t mask = 0x8040;
#else
    intptr_t mask = 0x8000;
#endif
#else
    intptr_t mask = 0x0000;
#endif
    getFpStatusRegister();
    setFpStatusRegister(fpsr | mask);
  }

  ~ScopedNoDenormals() noexcept { setFpStatusRegister(fpsr); }
};

#define AVOIDDENORMALS ScopedNoDenormals ftz_scope;

#endif

/************************** END dsp.h **************************/
/************************** BEGIN UI.h *****************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ********************************************************************/

#ifndef __UI_H__
#define __UI_H__

#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif

/*******************************************************************************
 * UI : Faust DSP User Interface
 * User Interface as expected by the buildUserInterface() method of a DSP.
 * This abstract class contains only the method that the Faust compiler can
 * generate to describe a DSP user interface.
 ******************************************************************************/

struct Soundfile;

template <typename REAL>
struct FAUST_API UIReal {
  UIReal() {}
  virtual ~UIReal() {}

  // -- widget's layouts

  virtual void openTabBox(const char* label) = 0;
  virtual void openHorizontalBox(const char* label) = 0;
  virtual void openVerticalBox(const char* label) = 0;
  virtual void closeBox() = 0;

  // -- active widgets

  virtual void addButton(const char* label, REAL* zone) = 0;
  virtual void addCheckButton(const char* label, REAL* zone) = 0;
  virtual void addVerticalSlider(const char* label, REAL* zone, REAL init,
                                 REAL min, REAL max, REAL step) = 0;
  virtual void addHorizontalSlider(const char* label, REAL* zone, REAL init,
                                   REAL min, REAL max, REAL step) = 0;
  virtual void addNumEntry(const char* label, REAL* zone, REAL init, REAL min,
                           REAL max, REAL step) = 0;

  // -- passive widgets

  virtual void addHorizontalBargraph(const char* label, REAL* zone, REAL min,
                                     REAL max) = 0;
  virtual void addVerticalBargraph(const char* label, REAL* zone, REAL min,
                                   REAL max) = 0;

  // -- soundfiles

  virtual void addSoundfile(const char* label, const char* filename,
                            Soundfile** sf_zone) = 0;

  // -- metadata declarations

  virtual void declare(REAL* /*zone*/, const char* /*key*/,
                       const char* /*val*/) {}

  // To be used by LLVM client
  virtual int sizeOfFAUSTFLOAT() { return sizeof(FAUSTFLOAT); }
};

struct FAUST_API UI : public UIReal<FAUSTFLOAT> {
  UI() {}
  virtual ~UI() {}
};

#endif
/**************************  END  UI.h **************************/
/************************** BEGIN misc.h *******************************
FAUST Architecture File
Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
---------------------------------------------------------------------
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

EXCEPTION : As a special exception, you may create a larger work
that contains this FAUST architecture section and distribute
that work under terms of your choice, so long as this FAUST
architecture section is not modified.
***************************************************************************/

#ifndef __misc__
#define __misc__

#include <string.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

/************************** BEGIN meta.h *******************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

#ifndef __meta__
#define __meta__

/**
 The base class of Meta handler to be used in dsp::metadata(Meta* m) method to
 retrieve (key, value) metadata.
 */
struct FAUST_API Meta {
  virtual ~Meta() {}
  virtual void declare(const char* key, const char* value) = 0;
};

#endif
/**************************  END  meta.h **************************/

struct MY_Meta : Meta, std::map<const char*, const char*> {
  void declare(const char* key, const char* value) { (*this)[key] = value; }
};

static int lsr(int x, int n) { return int(((unsigned int)x) >> n); }

static int int2pow2(int x) {
  int r = 0;
  while ((1 << r) < x) r++;
  return r;
}

static long lopt(char* argv[], const char* name, long def) {
  for (int i = 0; argv[i]; i++)
    if (!strcmp(argv[i], name)) return std::atoi(argv[i + 1]);
  return def;
}

static long lopt1(int argc, char* argv[], const char* longname,
                  const char* shortname, long def) {
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i - 1], shortname) == 0 ||
        strcmp(argv[i - 1], longname) == 0) {
      return atoi(argv[i]);
    }
  }
  return def;
}

static const char* lopts(char* argv[], const char* name, const char* def) {
  for (int i = 0; argv[i]; i++)
    if (!strcmp(argv[i], name)) return argv[i + 1];
  return def;
}

static const char* lopts1(int argc, char* argv[], const char* longname,
                          const char* shortname, const char* def) {
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i - 1], shortname) == 0 ||
        strcmp(argv[i - 1], longname) == 0) {
      return argv[i];
    }
  }
  return def;
}

static bool isopt(char* argv[], const char* name) {
  for (int i = 0; argv[i]; i++)
    if (!strcmp(argv[i], name)) return true;
  return false;
}

static std::string pathToContent(const std::string& path) {
  std::ifstream file(path.c_str(), std::ifstream::binary);

  file.seekg(0, file.end);
  int size = int(file.tellg());
  file.seekg(0, file.beg);

  // And allocate buffer to that a single line can be read...
  char* buffer = new char[size + 1];
  file.read(buffer, size);

  // Terminate the string
  buffer[size] = 0;
  std::string result = buffer;
  file.close();
  delete[] buffer;
  return result;
}

#endif

/**************************  END  misc.h **************************/

#ifdef SOUNDFILE
/************************** BEGIN SoundUI.h **************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ********************************************************************/

#ifndef __SoundUI_H__
#define __SoundUI_H__

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

/************************** BEGIN SimpleParser.h *********************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ********************************************************************/

#ifndef SIMPLEPARSER_H
#define SIMPLEPARSER_H

// ---------------------------------------------------------------------
//                          Simple Parser
// A parser returns true if it was able to parse what it is
// supposed to parse and advance the pointer. Otherwise it returns false
// and the pointer is not advanced so that another parser can be tried.
// ---------------------------------------------------------------------

#include <assert.h>
#include <ctype.h>
#include <stdio.h>  // We use the lighter fprintf code

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

struct itemInfo {
  std::string type;
  std::string label;
  std::string shortname;
  std::string address;
  std::string url;
  int index;
  double init;
  double fmin;
  double fmax;
  double step;
  std::vector<std::pair<std::string, std::string>> meta;

  itemInfo() : index(0), init(0.), fmin(0.), fmax(0.), step(0.) {}
};

// ---------------------------------------------------------------------
//                          Elementary parsers
// ---------------------------------------------------------------------

// Report a parsing error
static bool parseError(const char*& p, const char* errmsg) {
  fprintf(stderr, "Parse error : %s here : %s\n", errmsg, p);
  return true;
}

/**
 * @brief skipBlank : advance pointer p to the first non blank character
 * @param p the string to parse, then the remaining string
 */
static void skipBlank(const char*& p) {
  while (isspace(*p)) {
    p++;
  }
}

// Parse character x, but don't report error if fails
static bool tryChar(const char*& p, char x) {
  skipBlank(p);
  if (x == *p) {
    p++;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief parseChar : parse a specific character x
 * @param p the string to parse, then the remaining string
 * @param x the character to recognize
 * @return true if x was found at the begin of p
 */
static bool parseChar(const char*& p, char x) {
  skipBlank(p);
  if (x == *p) {
    p++;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief parseWord : parse a specific string w
 * @param p the string to parse, then the remaining string
 * @param w the string to recognize
 * @return true if string w was found at the begin of p
 */
static bool parseWord(const char*& p, const char* w) {
  skipBlank(p);
  const char* saved = p;  // to restore position if we fail
  while ((*w == *p) && (*w)) {
    ++w;
    ++p;
  }
  if (*w) {
    p = saved;
    return false;
  } else {
    return true;
  }
}

/**
 * @brief parseDouble : parse number [s]dddd[.dddd] or [s]d[.dddd][E|e][s][dddd]
 * and store the result in x
 * @param p the string to parse, then the remaining string
 * @param x the float number found if any
 * @return true if a float number was found at the begin of p
 */
static bool parseDouble(const char*& p, double& x) {
  double sign = 1.0;     // sign of the number
  double ipart = 0;      // integral part of the number
  double dpart = 0;      // decimal part of the number before division
  double dcoef = 1.0;    // division factor for the decimal part
  double expsign = 1.0;  // sign of the E|e part
  double expcoef = 0.0;  // multiplication factor of E|e part

  bool valid = false;  // true if the number contains at least one digit

  skipBlank(p);
  const char* saved = p;  // to restore position if we fail

  // Sign
  if (parseChar(p, '+')) {
    sign = 1.0;
  } else if (parseChar(p, '-')) {
    sign = -1.0;
  }

  // Integral part
  while (isdigit(*p)) {
    valid = true;
    ipart = ipart * 10 + (*p - '0');
    p++;
  }

  // Possible decimal part
  if (parseChar(p, '.')) {
    while (isdigit(*p)) {
      valid = true;
      dpart = dpart * 10 + (*p - '0');
      dcoef *= 10.0;
      p++;
    }
  }

  // Possible E|e part
  if (parseChar(p, 'E') || parseChar(p, 'e')) {
    if (parseChar(p, '+')) {
      expsign = 1.0;
    } else if (parseChar(p, '-')) {
      expsign = -1.0;
    }
    while (isdigit(*p)) {
      expcoef = expcoef * 10 + (*p - '0');
      p++;
    }
  }

  if (valid) {
    x = (sign * (ipart + dpart / dcoef)) * std::pow(10.0, expcoef * expsign);
  } else {
    p = saved;
  }
  return valid;
}

/**
 * @brief parseString, parse an arbitrary quoted string q...q and store the
 * result in s
 * @param p the string to parse, then the remaining string
 * @param quote the character used to quote the string
 * @param s the (unquoted) string found if any
 * @return true if a string was found at the begin of p
 */
static bool parseString(const char*& p, char quote, std::string& s) {
  std::string str;
  skipBlank(p);

  const char* saved = p;  // to restore position if we fail
  if (*p++ == quote) {
    while ((*p != 0) && (*p != quote)) {
      str += *p++;
    }
    if (*p++ == quote) {
      s = str;
      return true;
    }
  }
  p = saved;
  return false;
}

/**
 * @brief parseSQString, parse a single quoted string '...' and store the result
 * in s
 * @param p the string to parse, then the remaining string
 * @param s the (unquoted) string found if any
 * @return true if a string was found at the begin of p
 */
static bool parseSQString(const char*& p, std::string& s) {
  return parseString(p, '\'', s);
}

/**
 * @brief parseDQString, parse a double quoted string "..." and store the result
 * in s
 * @param p the string to parse, then the remaining string
 * @param s the (unquoted) string found if any
 * @return true if a string was found at the begin of p
 */
static bool parseDQString(const char*& p, std::string& s) {
  return parseString(p, '"', s);
}

// ---------------------------------------------------------------------
//
//                          IMPLEMENTATION
//
// ---------------------------------------------------------------------

/**
 * @brief parseMenuItem, parse a menu item ...'low':440.0...
 * @param p the string to parse, then the remaining string
 * @param name the name found
 * @param value the value found
 * @return true if a nemu item was found
 */
static bool parseMenuItem(const char*& p, std::string& name, double& value) {
  const char* saved = p;  // to restore position if we fail
  if (parseSQString(p, name) && parseChar(p, ':') && parseDouble(p, value)) {
    return true;
  } else {
    p = saved;
    return false;
  }
}

static bool parseMenuItem2(const char*& p, std::string& name) {
  const char* saved = p;  // to restore position if we fail
  // single quoted
  if (parseSQString(p, name)) {
    return true;
  } else {
    p = saved;
    return false;
  }
}

/**
 * @brief parseMenuList, parse a menu list {'low' : 440.0; 'mid' : 880.0; 'hi' :
 * 1760.0}...
 * @param p the string to parse, then the remaining string
 * @param names the vector of names found
 * @param values the vector of values found
 * @return true if a menu list was found
 */
static bool parseMenuList(const char*& p, std::vector<std::string>& names,
                          std::vector<double>& values) {
  std::vector<std::string> tmpnames;
  std::vector<double> tmpvalues;
  const char* saved = p;  // to restore position if we fail

  if (parseChar(p, '{')) {
    do {
      std::string n;
      double v;
      if (parseMenuItem(p, n, v)) {
        tmpnames.push_back(n);
        tmpvalues.push_back(v);
      } else {
        p = saved;
        return false;
      }
    } while (parseChar(p, ';'));
    if (parseChar(p, '}')) {
      // we suceeded
      names = tmpnames;
      values = tmpvalues;
      return true;
    }
  }
  p = saved;
  return false;
}

static bool parseMenuList2(const char*& p, std::vector<std::string>& names,
                           bool debug) {
  std::vector<std::string> tmpnames;
  const char* saved = p;  // to restore position if we fail

  if (parseChar(p, '{')) {
    do {
      std::string n;
      if (parseMenuItem2(p, n)) {
        tmpnames.push_back(n);
      } else {
        goto error;
      }
    } while (parseChar(p, ';'));
    if (parseChar(p, '}')) {
      // we suceeded
      names = tmpnames;
      return true;
    }
  }

error:
  if (debug) {
    fprintf(stderr, "parseMenuList2 : (%s) is not a valid list !\n", p);
  }
  p = saved;
  return false;
}

/// ---------------------------------------------------------------------
// Parse list of strings
/// ---------------------------------------------------------------------
static bool parseList(const char*& p, std::vector<std::string>& items) {
  const char* saved = p;  // to restore position if we fail
  if (parseChar(p, '[')) {
    do {
      std::string item;
      if (!parseDQString(p, item)) {
        p = saved;
        return false;
      }
      items.push_back(item);
    } while (tryChar(p, ','));
    return parseChar(p, ']');
  } else {
    p = saved;
    return false;
  }
}

static bool parseMetaData(const char*& p,
                          std::map<std::string, std::string>& metadatas) {
  const char* saved = p;  // to restore position if we fail
  std::string metaKey, metaValue;
  if (parseChar(p, ':') && parseChar(p, '[')) {
    do {
      if (parseChar(p, '{') && parseDQString(p, metaKey) && parseChar(p, ':') &&
          parseDQString(p, metaValue) && parseChar(p, '}')) {
        metadatas[metaKey] = metaValue;
      }
    } while (tryChar(p, ','));
    return parseChar(p, ']');
  } else {
    p = saved;
    return false;
  }
}

static bool parseItemMetaData(
    const char*& p,
    std::vector<std::pair<std::string, std::string>>& metadatas) {
  const char* saved = p;  // to restore position if we fail
  std::string metaKey, metaValue;
  if (parseChar(p, ':') && parseChar(p, '[')) {
    do {
      if (parseChar(p, '{') && parseDQString(p, metaKey) && parseChar(p, ':') &&
          parseDQString(p, metaValue) && parseChar(p, '}')) {
        metadatas.push_back(std::make_pair(metaKey, metaValue));
      }
    } while (tryChar(p, ','));
    return parseChar(p, ']');
  } else {
    p = saved;
    return false;
  }
}

// ---------------------------------------------------------------------
// Parse metadatas of the interface:
// "name" : "...", "inputs" : "...", "outputs" : "...", ...
// and store the result as key/value
/// ---------------------------------------------------------------------
static bool parseGlobalMetaData(const char*& p, std::string& key,
                                std::string& value, double& dbl,
                                std::map<std::string, std::string>& metadatas,
                                std::vector<std::string>& items) {
  const char* saved = p;  // to restore position if we fail
  if (parseDQString(p, key)) {
    if (key == "meta") {
      return parseMetaData(p, metadatas);
    } else {
      return parseChar(p, ':') && (parseDQString(p, value) ||
                                   parseList(p, items) || parseDouble(p, dbl));
    }
  } else {
    p = saved;
    return false;
  }
}

// ---------------------------------------------------------------------
// Parse gui:
// "type" : "...", "label" : "...", "address" : "...", ...
// and store the result in uiItems Vector
/// ---------------------------------------------------------------------
static bool parseUI(const char*& p, std::vector<itemInfo>& uiItems,
                    int& numItems) {
  const char* saved = p;  // to restore position if we fail
  if (parseChar(p, '{')) {
    std::string label;
    std::string value;
    double dbl = 0;

    do {
      if (parseDQString(p, label)) {
        if (label == "type") {
          if (uiItems.size() != 0) {
            numItems++;
          }
          if (parseChar(p, ':') && parseDQString(p, value)) {
            itemInfo item;
            item.type = value;
            uiItems.push_back(item);
          }
        }

        else if (label == "label") {
          if (parseChar(p, ':') && parseDQString(p, value)) {
            uiItems[numItems].label = value;
          }
        }

        else if (label == "shortname") {
          if (parseChar(p, ':') && parseDQString(p, value)) {
            uiItems[numItems].shortname = value;
          }
        }

        else if (label == "address") {
          if (parseChar(p, ':') && parseDQString(p, value)) {
            uiItems[numItems].address = value;
          }
        }

        else if (label == "url") {
          if (parseChar(p, ':') && parseDQString(p, value)) {
            uiItems[numItems].url = value;
          }
        }

        else if (label == "index") {
          if (parseChar(p, ':') && parseDouble(p, dbl)) {
            uiItems[numItems].index = int(dbl);
          }
        }

        else if (label == "meta") {
          if (!parseItemMetaData(p, uiItems[numItems].meta)) {
            return false;
          }
        }

        else if (label == "init") {
          if (parseChar(p, ':') && parseDouble(p, dbl)) {
            uiItems[numItems].init = dbl;
          }
        }

        else if (label == "min") {
          if (parseChar(p, ':') && parseDouble(p, dbl)) {
            uiItems[numItems].fmin = dbl;
          }
        }

        else if (label == "max") {
          if (parseChar(p, ':') && parseDouble(p, dbl)) {
            uiItems[numItems].fmax = dbl;
          }
        }

        else if (label == "step") {
          if (parseChar(p, ':') && parseDouble(p, dbl)) {
            uiItems[numItems].step = dbl;
          }
        }

        else if (label == "items") {
          if (parseChar(p, ':') && parseChar(p, '[')) {
            do {
              if (!parseUI(p, uiItems, numItems)) {
                p = saved;
                return false;
              }
            } while (tryChar(p, ','));
            if (parseChar(p, ']')) {
              itemInfo item;
              item.type = "close";
              uiItems.push_back(item);
              numItems++;
            }
          }

        } else {
          fprintf(stderr, "Parse error unknown : %s \n", label.c_str());
          assert(false);
        }
      } else {
        p = saved;
        return false;
      }

    } while (tryChar(p, ','));

    return parseChar(p, '}');
  } else {
    return true;  // "items": [] is valid
  }
}

// ---------------------------------------------------------------------
// Parse full JSON record describing a JSON/Faust interface :
// {"metadatas": "...", "ui": [{ "type": "...", "label": "...", "items": [...],
// "address": "...","init": "...", "min": "...", "max": "...","step": "..."}]}
//
// and store the result in map Metadatas and vector containing the items of the
// interface. Returns true if parsing was successfull.
/// ---------------------------------------------------------------------
static bool parseJson(
    const char*& p,
    std::map<std::string, std::pair<std::string, double>>& metaDatas0,
    std::map<std::string, std::string>& metaDatas1,
    std::map<std::string, std::vector<std::string>>& metaDatas2,
    std::vector<itemInfo>& uiItems) {
  parseChar(p, '{');

  do {
    std::string key;
    std::string value;
    double dbl = 0;
    std::vector<std::string> items;
    if (parseGlobalMetaData(p, key, value, dbl, metaDatas1, items)) {
      if (key != "meta") {
        // keep "name", "inputs", "outputs" key/value pairs
        if (items.size() > 0) {
          metaDatas2[key] = items;
          items.clear();
        } else if (value != "") {
          metaDatas0[key].first = value;
        } else {
          metaDatas0[key].second = dbl;
        }
      }
    } else if (key == "ui") {
      int numItems = 0;
      parseChar(p, '[') && parseUI(p, uiItems, numItems);
    }
  } while (tryChar(p, ','));

  return parseChar(p, '}');
}

#endif  // SIMPLEPARSER_H
/**************************  END  SimpleParser.h **************************/
/************************** BEGIN DecoratorUI.h **************************
 FAUST Architecture File
Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
---------------------------------------------------------------------
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

EXCEPTION : As a special exception, you may create a larger work
that contains this FAUST architecture section and distribute
that work under terms of your choice, so long as this FAUST
architecture section is not modified.
*************************************************************************/

#ifndef Decorator_UI_H
#define Decorator_UI_H

//----------------------------------------------------------------
//  Generic UI empty implementation
//----------------------------------------------------------------

class FAUST_API GenericUI : public UI {
 public:
  GenericUI() {}
  virtual ~GenericUI() {}

  // -- widget's layouts
  virtual void openTabBox(const char* label) {}
  virtual void openHorizontalBox(const char* label) {}
  virtual void openVerticalBox(const char* label) {}
  virtual void closeBox() {}

  // -- active widgets
  virtual void addButton(const char* label, FAUSTFLOAT* zone) {}
  virtual void addCheckButton(const char* label, FAUSTFLOAT* zone) {}
  virtual void addVerticalSlider(const char* label, FAUSTFLOAT* zone,
                                 FAUSTFLOAT init, FAUSTFLOAT min,
                                 FAUSTFLOAT max, FAUSTFLOAT step) {}
  virtual void addHorizontalSlider(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT init, FAUSTFLOAT min,
                                   FAUSTFLOAT max, FAUSTFLOAT step) {}
  virtual void addNumEntry(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT init,
                           FAUSTFLOAT min, FAUSTFLOAT max, FAUSTFLOAT step) {}

  // -- passive widgets
  virtual void addHorizontalBargraph(const char* label, FAUSTFLOAT* zone,
                                     FAUSTFLOAT min, FAUSTFLOAT max) {}
  virtual void addVerticalBargraph(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT min, FAUSTFLOAT max) {}

  // -- soundfiles
  virtual void addSoundfile(const char* label, const char* soundpath,
                            Soundfile** sf_zone) {}

  virtual void declare(FAUSTFLOAT* zone, const char* key, const char* val) {}
};

//----------------------------------------------------------------
//  Generic UI decorator
//----------------------------------------------------------------

class FAUST_API DecoratorUI : public UI {
 protected:
  UI* fUI;

 public:
  DecoratorUI(UI* ui = 0) : fUI(ui) {}
  virtual ~DecoratorUI() { delete fUI; }

  // -- widget's layouts
  virtual void openTabBox(const char* label) { fUI->openTabBox(label); }
  virtual void openHorizontalBox(const char* label) {
    fUI->openHorizontalBox(label);
  }
  virtual void openVerticalBox(const char* label) {
    fUI->openVerticalBox(label);
  }
  virtual void closeBox() { fUI->closeBox(); }

  // -- active widgets
  virtual void addButton(const char* label, FAUSTFLOAT* zone) {
    fUI->addButton(label, zone);
  }
  virtual void addCheckButton(const char* label, FAUSTFLOAT* zone) {
    fUI->addCheckButton(label, zone);
  }
  virtual void addVerticalSlider(const char* label, FAUSTFLOAT* zone,
                                 FAUSTFLOAT init, FAUSTFLOAT min,
                                 FAUSTFLOAT max, FAUSTFLOAT step) {
    fUI->addVerticalSlider(label, zone, init, min, max, step);
  }
  virtual void addHorizontalSlider(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT init, FAUSTFLOAT min,
                                   FAUSTFLOAT max, FAUSTFLOAT step) {
    fUI->addHorizontalSlider(label, zone, init, min, max, step);
  }
  virtual void addNumEntry(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT init,
                           FAUSTFLOAT min, FAUSTFLOAT max, FAUSTFLOAT step) {
    fUI->addNumEntry(label, zone, init, min, max, step);
  }

  // -- passive widgets
  virtual void addHorizontalBargraph(const char* label, FAUSTFLOAT* zone,
                                     FAUSTFLOAT min, FAUSTFLOAT max) {
    fUI->addHorizontalBargraph(label, zone, min, max);
  }
  virtual void addVerticalBargraph(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT min, FAUSTFLOAT max) {
    fUI->addVerticalBargraph(label, zone, min, max);
  }

  // -- soundfiles
  virtual void addSoundfile(const char* label, const char* filename,
                            Soundfile** sf_zone) {
    fUI->addSoundfile(label, filename, sf_zone);
  }

  virtual void declare(FAUSTFLOAT* zone, const char* key, const char* val) {
    fUI->declare(zone, key, val);
  }
};

// Defined here to simplify header #include inclusion
class FAUST_API SoundUIInterface : public GenericUI {};

#endif
/**************************  END  DecoratorUI.h **************************/

#if defined(__APPLE__) && !defined(__VCVRACK__) && !defined(JUCE_32BIT) && \
    !defined(JUCE_64BIT)
#include <CoreFoundation/CFBundle.h>
#endif

// Always included otherwise -i mode later on will not always include it (with
// the conditional includes)
/************************** BEGIN Soundfile.h **************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ********************************************************************/

#ifndef __Soundfile__
#define __Soundfile__

#include <string.h>

#include <string>
#include <vector>

#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif

#define BUFFER_SIZE 1024
#define SAMPLE_RATE 44100
#define MAX_CHAN 64
#define MAX_SOUNDFILE_PARTS 256

#ifdef _MSC_VER
#define PRE_PACKED_STRUCTURE __pragma(pack(push, 1))
#define POST_PACKED_STRUCTURE \
  ;                           \
  __pragma(pack(pop))
#else
#define PRE_PACKED_STRUCTURE
#define POST_PACKED_STRUCTURE __attribute__((__packed__))
#endif

/*
 The soundfile structure to be used by the DSP code. Soundfile has a
 MAX_SOUNDFILE_PARTS parts (even a single soundfile or an empty soundfile). The
 fLength, fOffset and fSR fields are filled accordingly by repeating the actual
 parts if needed. The fBuffers contains MAX_CHAN non-interleaved arrays of
 samples.

 It has to be 'packed' to that the LLVM backend can correctly access it.

 Index computation:
    - p is the current part number [0..MAX_SOUNDFILE_PARTS-1] (must be proved by
 the type system)
    - i is the current position in the part. It will be constrained between
 [0..length]
    - idx(p,i) = fOffset[p] + max(0, min(i, fLength[p]));
*/

PRE_PACKED_STRUCTURE
struct Soundfile {
  void* fBuffers;  // will correspond to a double** or float** pointer chosen at
                   // runtime
  int* fLength;    // length of each part (so fLength[P] contains the length in
                   // frames of part P)
  int* fSR;  // sample rate of each part (so fSR[P] contains the SR of part P)
  int* fOffset;    // offset of each part in the global buffer (so fOffset[P]
                   // contains the offset in frames of part P)
  int fChannels;   // max number of channels of all concatenated files
  int fParts;      // the total number of loaded parts
  bool fIsDouble;  // keep the sample format (float or double)

  Soundfile(int cur_chan, int length, int max_chan, int total_parts,
            bool is_double) {
    fLength = new int[MAX_SOUNDFILE_PARTS];
    fSR = new int[MAX_SOUNDFILE_PARTS];
    fOffset = new int[MAX_SOUNDFILE_PARTS];
    fIsDouble = is_double;
    fChannels = cur_chan;
    fParts = total_parts;
    if (fIsDouble) {
      fBuffers = allocBufferReal<double>(cur_chan, length, max_chan);
    } else {
      fBuffers = allocBufferReal<float>(cur_chan, length, max_chan);
    }
  }

  template <typename REAL>
  void* allocBufferReal(int cur_chan, int length, int max_chan) {
    REAL** buffers = new REAL*[max_chan];
    for (int chan = 0; chan < cur_chan; chan++) {
      buffers[chan] = new REAL[length];
      memset(buffers[chan], 0, sizeof(REAL) * length);
    }
    return buffers;
  }

  void copyToOut(int size, int channels, int max_channels, int offset,
                 void* buffer) {
    if (fIsDouble) {
      copyToOutReal<double>(size, channels, max_channels, offset, buffer);
    } else {
      copyToOutReal<float>(size, channels, max_channels, offset, buffer);
    }
  }

  void shareBuffers(int cur_chan, int max_chan) {
    // Share the same buffers for all other channels so that we have max_chan
    // channels available
    if (fIsDouble) {
      for (int chan = cur_chan; chan < max_chan; chan++) {
        static_cast<double**>(fBuffers)[chan] =
            static_cast<double**>(fBuffers)[chan % cur_chan];
      }
    } else {
      for (int chan = cur_chan; chan < max_chan; chan++) {
        static_cast<float**>(fBuffers)[chan] =
            static_cast<float**>(fBuffers)[chan % cur_chan];
      }
    }
  }

  template <typename REAL>
  void copyToOutReal(int size, int channels, int max_channels, int offset,
                     void* buffer) {
    for (int sample = 0; sample < size; sample++) {
      for (int chan = 0; chan < channels; chan++) {
        static_cast<REAL**>(fBuffers)[chan][offset + sample] =
            static_cast<REAL*>(buffer)[sample * max_channels + chan];
      }
    }
  }

  template <typename REAL>
  void getBuffersOffsetReal(void* buffers, int offset) {
    for (int chan = 0; chan < fChannels; chan++) {
      static_cast<REAL**>(buffers)[chan] =
          &(static_cast<REAL**>(fBuffers))[chan][offset];
    }
  }

  void emptyFile(int part, int& offset) {
    fLength[part] = BUFFER_SIZE;
    fSR[part] = SAMPLE_RATE;
    fOffset[part] = offset;
    // Update offset
    offset += fLength[part];
  }

  ~Soundfile() {
    // Free the real channels only
    if (fIsDouble) {
      for (int chan = 0; chan < fChannels; chan++) {
        delete[] static_cast<double**>(fBuffers)[chan];
      }
      delete[] static_cast<double**>(fBuffers);
    } else {
      for (int chan = 0; chan < fChannels; chan++) {
        delete[] static_cast<float**>(fBuffers)[chan];
      }
      delete[] static_cast<float**>(fBuffers);
    }
    delete[] fLength;
    delete[] fSR;
    delete[] fOffset;
  }

  typedef std::vector<std::string> Directories;

} POST_PACKED_STRUCTURE;

/*
 The generic soundfile reader.
 */

class SoundfileReader {
 protected:
  int fDriverSR;

  // Check if a soundfile exists and return its real path_name
  std::string checkFile(const Soundfile::Directories& sound_directories,
                        const std::string& file_name) {
    if (checkFile(file_name)) {
      return file_name;
    } else {
      for (size_t i = 0; i < sound_directories.size(); i++) {
        std::string path_name = sound_directories[i] + "/" + file_name;
        if (checkFile(path_name)) {
          return path_name;
        }
      }
      return "";
    }
  }

  bool isResampling(int sample_rate) {
    return (fDriverSR > 0 && fDriverSR != sample_rate);
  }

  // To be implemented by subclasses

  /**
   * Check the availability of a sound resource.
   *
   * @param path_name - the name of the file, or sound resource identified this
   * way
   *
   * @return true if the sound resource is available, false otherwise.
   */
  virtual bool checkFile(const std::string& path_name) = 0;

  /**
   * Check the availability of a sound resource.
   *
   * @param buffer - the sound buffer
   * @param size - the sound buffer length
   *
   * @return true if the sound resource is available, false otherwise.
   */

  virtual bool checkFile(unsigned char* buffer, size_t size) { return true; }

  /**
   * Get the channels and length values of the given sound resource.
   *
   * @param path_name - the name of the file, or sound resource identified this
   * way
   * @param channels - the channels value to be filled with the sound resource
   * number of channels
   * @param length - the length value to be filled with the sound resource
   * length in frames
   *
   */
  virtual void getParamsFile(const std::string& path_name, int& channels,
                             int& length) = 0;

  /**
   * Get the channels and length values of the given sound resource.
   *
   * @param buffer - the sound buffer
   * @param size - the sound buffer length
   * @param channels - the channels value to be filled with the sound resource
   * number of channels
   * @param length - the length value to be filled with the sound resource
   * length in frames
   *
   */
  virtual void getParamsFile(unsigned char* buffer, size_t size, int& channels,
                             int& length) {}

  /**
   * Read one sound resource and fill the 'soundfile' structure accordingly
   *
   * @param soundfile - the soundfile to be filled
   * @param path_name - the name of the file, or sound resource identified this
   * way
   * @param part - the part number to be filled in the soundfile
   * @param offset - the offset value to be incremented with the actual sound
   * resource length in frames
   * @param max_chan - the maximum number of mono channels to fill
   *
   */
  virtual void readFile(Soundfile* soundfile, const std::string& path_name,
                        int part, int& offset, int max_chan) = 0;

  /**
   * Read one sound resource and fill the 'soundfile' structure accordingly
   *
   * @param soundfile - the soundfile to be filled
   * @param buffer - the sound buffer
   * @param size - the sound buffer length
   * @param part - the part number to be filled in the soundfile
   * @param offset - the offset value to be incremented with the actual sound
   * resource length in frames
   * @param max_chan - the maximum number of mono channels to fill
   *
   */
  virtual void readFile(Soundfile* soundfile, unsigned char* buffer,
                        size_t size, int part, int& offset, int max_chan) {}

 public:
  SoundfileReader() {}
  virtual ~SoundfileReader() {}

  void setSampleRate(int sample_rate) { fDriverSR = sample_rate; }

  Soundfile* createSoundfile(const std::vector<std::string>& path_name_list,
                             int max_chan, bool is_double) {
    try {
      int cur_chan = 1;  // At least one channel
      int total_length = 0;

      // Compute total length and channels max of all files
      for (size_t i = 0; i < path_name_list.size(); i++) {
        int chan, length;
        if (path_name_list[i] == "__empty_sound__") {
          length = BUFFER_SIZE;
          chan = 1;
        } else {
          getParamsFile(path_name_list[i], chan, length);
        }
        cur_chan = std::max<int>(cur_chan, chan);
        total_length += length;
      }

      // Complete with empty parts
      total_length +=
          (MAX_SOUNDFILE_PARTS - path_name_list.size()) * BUFFER_SIZE;

      // Create the soundfile
      Soundfile* soundfile = new Soundfile(cur_chan, total_length, max_chan,
                                           path_name_list.size(), is_double);

      // Init offset
      int offset = 0;

      // Read all files
      for (size_t i = 0; i < path_name_list.size(); i++) {
        if (path_name_list[i] == "__empty_sound__") {
          soundfile->emptyFile(i, offset);
        } else {
          readFile(soundfile, path_name_list[i], i, offset, max_chan);
        }
      }

      // Complete with empty parts
      for (size_t i = path_name_list.size(); i < MAX_SOUNDFILE_PARTS; i++) {
        soundfile->emptyFile(i, offset);
      }

      // Share the same buffers for all other channels so that we have max_chan
      // channels available
      soundfile->shareBuffers(cur_chan, max_chan);
      return soundfile;

    } catch (...) {
      return nullptr;
    }
  }

  // Check if all soundfiles exist and return their real path_name
  std::vector<std::string> checkFiles(
      const Soundfile::Directories& sound_directories,
      const std::vector<std::string>& file_name_list) {
    std::vector<std::string> path_name_list;
    for (size_t i = 0; i < file_name_list.size(); i++) {
      std::string path_name = checkFile(sound_directories, file_name_list[i]);
      // If 'path_name' is not found, it is replaced by an empty sound (=
      // silence)
      path_name_list.push_back((path_name == "") ? "__empty_sound__"
                                                 : path_name);
    }
    return path_name_list;
  }
};

#endif
/**************************  END  Soundfile.h **************************/

#if defined(JUCE_32BIT) || defined(JUCE_64BIT)
/************************** BEGIN JuceReader.h **************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

#ifndef __JuceReader__
#define __JuceReader__

#include <assert.h>

#include "../JuceLibraryCode/JuceHeader.h"

struct JuceReader : public SoundfileReader {
  juce::AudioFormatManager fFormatManager;

  JuceReader() { fFormatManager.registerBasicFormats(); }
  virtual ~JuceReader() {}

  bool checkFile(const std::string& path_name) override {
    juce::File file =
        juce::File::getCurrentWorkingDirectory().getChildFile(path_name);
    if (file.existsAsFile()) {
      return true;
    } else {
      // std::cerr << "ERROR : cannot open '" << path_name << "'" << std::endl;
      return false;
    }
  }

  void getParamsFile(const std::string& path_name, int& channels,
                     int& length) override {
    std::unique_ptr<juce::AudioFormatReader> formatReader(
        fFormatManager.createReaderFor(
            juce::File::getCurrentWorkingDirectory().getChildFile(path_name)));
    channels = int(formatReader->numChannels);
    length = int(formatReader->lengthInSamples);
  }

  void readFile(Soundfile* soundfile, const std::string& path_name, int part,
                int& offset, int max_chan) override {
    std::unique_ptr<juce::AudioFormatReader> formatReader(
        fFormatManager.createReaderFor(
            juce::File::getCurrentWorkingDirectory().getChildFile(path_name)));

    soundfile->fLength[part] = int(formatReader->lengthInSamples);
    soundfile->fSR[part] = int(formatReader->sampleRate);
    soundfile->fOffset[part] = offset;

    void* buffers;
    if (soundfile->fIsDouble) {
      buffers = alloca(soundfile->fChannels * sizeof(double*));
      soundfile->getBuffersOffsetReal<double>(buffers, offset);
    } else {
      buffers = alloca(soundfile->fChannels * sizeof(float*));
      soundfile->getBuffersOffsetReal<float>(buffers, offset);
    }

    if (formatReader->read(reinterpret_cast<int* const*>(buffers),
                           int(formatReader->numChannels), 0,
                           int(formatReader->lengthInSamples), false)) {
      // Possibly convert samples
      if (!formatReader->usesFloatingPointData) {
        for (int chan = 0; chan < int(formatReader->numChannels); ++chan) {
          if (soundfile->fIsDouble) {
            // TODO
          } else {
            float* buffer = &(static_cast<float**>(
                soundfile->fBuffers))[chan][soundfile->fOffset[part]];
            juce::FloatVectorOperations::convertFixedToFloat(
                buffer, reinterpret_cast<const int*>(buffer), 1.0f / 0x7fffffff,
                int(formatReader->lengthInSamples));
          }
        }
      }

    } else {
      std::cerr << "Error reading the file : " << path_name << std::endl;
    }

    // Update offset
    offset += soundfile->fLength[part];
  }
};

#endif
/**************************  END  JuceReader.h **************************/
static JuceReader gReader;
#elif defined(DAISY) || defined(SUPERCOLLIDER)
/************************** BEGIN WaveReader.h **************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ********************************************************************/

#ifndef __WaveReader__
#define __WaveReader__

#include <assert.h>
#include <stdio.h>
#include <string.h>

// WAVE file description
typedef struct {
  // The canonical WAVE format starts with the RIFF header

  /**
   Variable: chunk_id
   Contains the letters "RIFF" in ASCII form (0x52494646 big-endian form).
   **/
  int chunk_id;

  /**
   Variable: chunk_size
   36 + SubChunk2Size, or more precisely: 4 + (8 + SubChunk1Size) + (8 +
   SubChunk2Size) This is the size of the rest of the chunk following this
   number. This is the size of the entire file in bytes minus 8 bytes for the
   two fields not included in this count: ChunkID and ChunkSize.
   **/
  int chunk_size;

  /**
   Variable: format
   Contains the letters "WAVE" (0x57415645 big-endian form).
   **/
  int format;

  // The "WAVE" format consists of two subchunks: "fmt " and "data":
  // The "fmt " subchunk describes the sound data's format:

  /**
   Variable: subchunk_1_id
   Contains the letters "fmt " (0x666d7420 big-endian form).
   **/
  int subchunk_1_id;

  /**
   Variable: subchunk_1_size
   16 for PCM. This is the size of the rest of the Subchunk which follows this
   number.
   **/
  int subchunk_1_size;

  /**
   Variable: audio_format
   PCM = 1 (i.e. Linear quantization) Values other than 1 indicate some form of
   compression.
   **/
  short audio_format;

  /**
   Variable: num_channels
   Mono = 1, Stereo = 2, etc.
   **/
  short num_channels;

  /**
   Variable: sample_rate
   8000, 44100, etc.
   **/
  int sample_rate;

  /**
   Variable: byte_rate
   == SampleRate * NumChannels * BitsPerSample/8
   **/
  int byte_rate;

  /**
   Variable: block_align
   == NumChannels * BitsPerSample/8
   The number of bytes for one sample including all channels. I wonder what
   happens when this number isn't an integer?
   **/
  short block_align;

  /**
   Variable: bits_per_sample
   8 bits = 8, 16 bits = 16, etc.
   **/
  short bits_per_sample;

  /**
   Here should come some extra parameters which i will avoid.
   **/

  // The "data" subchunk contains the size of the data and the actual sound:

  /**
   Variable: subchunk_2_id
   Contains the letters "data" (0x64617461 big-endian form).
   **/
  int subchunk_2_id;

  /**
   Variable: subchunk_2_size
   == NumSamples * NumChannels * BitsPerSample/8
   This is the number of bytes in the data. You can also think of this as the
   size of the read of the subchunk following this number.
   **/
  int subchunk_2_size;

  /**
   Variable: data
   The actual sound data.
   **/
  char* data;

} wave_t;

// Base reader
struct Reader {
  wave_t* fWave;

  inline int is_big_endian() {
    int a = 1;
    return !((char*)&a)[0];
  }

  inline int convert_to_int(char* buffer, int len) {
    int a = 0;
    if (!is_big_endian()) {
      for (int i = 0; i < len; i++) {
        ((char*)&a)[i] = buffer[i];
      }
    } else {
      for (int i = 0; i < len; i++) {
        ((char*)&a)[3 - i] = buffer[i];
      }
    }
    return a;
  }

  Reader() { fWave = (wave_t*)calloc(1, sizeof(wave_t)); }

  virtual ~Reader() {
    free(fWave->data);
    free(fWave);
  }

  bool load_wave_header() {
    char buffer[4];

    read(buffer, 4);
    if (strncmp(buffer, "RIFF", 4) != 0) {
      fprintf(stderr, "This is not valid WAV file!\n");
      return false;
    }
    fWave->chunk_id = convert_to_int(buffer, 4);

    read(buffer, 4);
    fWave->chunk_size = convert_to_int(buffer, 4);

    read(buffer, 4);
    fWave->format = convert_to_int(buffer, 4);

    read(buffer, 4);
    fWave->subchunk_1_id = convert_to_int(buffer, 4);

    read(buffer, 4);
    fWave->subchunk_1_size = convert_to_int(buffer, 4);

    read(buffer, 2);
    fWave->audio_format = convert_to_int(buffer, 2);

    read(buffer, 2);
    fWave->num_channels = convert_to_int(buffer, 2);

    read(buffer, 4);
    fWave->sample_rate = convert_to_int(buffer, 4);

    read(buffer, 4);
    fWave->byte_rate = convert_to_int(buffer, 4);

    read(buffer, 2);
    fWave->block_align = convert_to_int(buffer, 2);

    read(buffer, 2);
    fWave->bits_per_sample = convert_to_int(buffer, 2);

    read(buffer, 4);
    if (strncmp(buffer, "data", 4) != 0) {
      read(buffer, 4);
      int _extra_size = convert_to_int(buffer, 4);
      char _extra_data[_extra_size];
      read(_extra_data, _extra_size);
      read(buffer, 4);
      fWave->subchunk_2_id = convert_to_int(buffer, 4);
    } else {
      fWave->subchunk_2_id = convert_to_int(buffer, 4);
    }

    read(buffer, 4);
    fWave->subchunk_2_size = convert_to_int(buffer, 4);
    return true;
  }

  void load_wave() {
    // Read sound data
    fWave->data = (char*)malloc(fWave->subchunk_2_size);
    read(fWave->data, fWave->subchunk_2_size);
  }

  virtual void read(char* buffer, unsigned int size) = 0;
};

struct FileReader : public Reader {
  FILE* fFile;

  FileReader(const std::string& file_path) {
    fFile = fopen(file_path.c_str(), "rb");
    if (!fFile) {
      fprintf(stderr, "FileReader : cannot open file!\n");
      throw -1;
    }
    if (!load_wave_header()) {
      fprintf(stderr, "FileReader : not a WAV file!\n");
      throw -1;
    }
  }

  virtual ~FileReader() { fclose(fFile); }

  void read(char* buffer, unsigned int size) { fread(buffer, 1, size, fFile); }
};

extern const uint8_t file_start[] asm("_binary_FILE_start");
extern const uint8_t file_end[] asm("_binary_FILE_end");

struct MemoryReader : public Reader {
  int fPos;
  const uint8_t* fStart;
  const uint8_t* fEnd;

  MemoryReader(const uint8_t* start, const uint8_t* end) : fPos(0) {
    fStart = start;
    fEnd = end;
    if (!load_wave_header()) {
      fprintf(stderr, "MemoryReader : not a WAV file!\n");
      throw -1;
    }
  }

  virtual ~MemoryReader() {}

  void read(char* buffer, unsigned int size) {
    memcpy(buffer, fStart + fPos, size);
    fPos += size;
  }
};

// Using a FileReader to implement SoundfileReader

struct WaveReader : public SoundfileReader {
  WaveReader() {}
  virtual ~WaveReader() {}

  bool checkFile(const std::string& path_name) override {
    try {
      FileReader reader(path_name);
      return true;
    } catch (...) {
      return false;
    }
  }

  void getParamsFile(const std::string& path_name, int& channels,
                     int& length) override {
    FileReader reader(path_name);
    channels = reader.fWave->num_channels;
    length = (reader.fWave->subchunk_2_size * 8) /
             (reader.fWave->num_channels * reader.fWave->bits_per_sample);
  }

  void readFile(Soundfile* soundfile, const std::string& path_name, int part,
                int& offset, int max_chan) override {
    FileReader reader(path_name);
    reader.load_wave();

    soundfile->fLength[part] =
        (reader.fWave->subchunk_2_size * 8) /
        (reader.fWave->num_channels * reader.fWave->bits_per_sample);
    soundfile->fSR[part] = reader.fWave->sample_rate;
    soundfile->fOffset[part] = offset;

    // Audio frames have to be written for each chan
    if (reader.fWave->bits_per_sample == 16) {
      float factor = 1.f / 32767.f;
      for (int sample = 0; sample < soundfile->fLength[part]; sample++) {
        short* frame =
            (short*)&reader.fWave->data[reader.fWave->block_align * sample];
        if (soundfile->fIsDouble) {
          for (int chan = 0; chan < reader.fWave->num_channels; chan++) {
            static_cast<double**>(soundfile->fBuffers)[chan][offset + sample] =
                frame[chan] * factor;
          }
        } else {
          for (int chan = 0; chan < reader.fWave->num_channels; chan++) {
            static_cast<float**>(soundfile->fBuffers)[chan][offset + sample] =
                frame[chan] * factor;
          }
        }
      }
    } else if (reader.fWave->bits_per_sample == 32) {
      fprintf(stderr, "readFile : not implemented\n");
    }

    // Update offset
    offset += soundfile->fLength[part];
  }
};

#endif
/**************************  END  WaveReader.h **************************/
static WaveReader gReader;
#elif defined(ESP32)
/************************** BEGIN Esp32Reader.h **************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 *************************************************************************/

#ifndef FAUST_ESP32READER_H
#define FAUST_ESP32READER_H

#include <stdio.h>

#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define TAG "Esp32Reader"

#define SD_PIN_NUM_MISO GPIO_NUM_2
#define SD_PIN_NUM_MOSI GPIO_NUM_15
#define SD_PIN_NUM_CLK GPIO_NUM_14
#define SD_PIN_NUM_CS GPIO_NUM_13

struct Esp32Reader : public WaveReader {
  void sdcard_init() {
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = SD_PIN_NUM_MISO;
    slot_config.gpio_mosi = SD_PIN_NUM_MOSI;
    slot_config.gpio_sck = SD_PIN_NUM_CLK;
    slot_config.gpio_cs = SD_PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP)
    // signals. Modify slot_config.gpio_cd and slot_config.gpio_wp if your board
    // has these signals.

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    // Use settings defined above to initialize SD card and mount FAT
    // filesystem. Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience
    // function. Please check its source code and implement error recovery when
    // developing production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config,
                                            &mount_config, &card);

    if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
        ESP_LOGE(TAG,
                 "Failed to mount filesystem. "
                 "If you want the card to be formatted, set "
                 "format_if_mount_failed = true.");
      } else {
        ESP_LOGE(TAG,
                 "Failed to initialize the card (%s). "
                 "Make sure SD card lines have pull-up resistors in place.",
                 esp_err_to_name(ret));
      }
      return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD card initialized");
  }

  Esp32Reader() { sdcard_init(); }

  // Access methods inherited from WaveReader
};

#endif  // FAUST_ESP32READER_H
/**************************  END  Esp32Reader.h **************************/
static Esp32Reader gReader;
#elif defined(MEMORY_READER)
/************************** BEGIN MemoryReader.h ************************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

#ifndef __MemoryReader__
#define __MemoryReader__

/*
 A 'MemoryReader' object can be used to prepare a set of sound resources in
 memory, to be used by SoundUI::addSoundfile.

 A Soundfile* object will have to be filled with a list of sound resources: the
 fLength, fOffset, fSampleRate and fBuffers fields have to be completed with the
 appropriate values, and will be accessed in the DSP object while running.
 *
 */

// To adapt for a real case use

#define SOUND_CHAN 2
#define SOUND_LENGTH 4096
#define SOUND_SR 44100

struct MemoryReader : public SoundfileReader {
  MemoryReader() {}
  virtual ~MemoryReader() {}

  /**
   * Check the availability of a sound resource.
   *
   * @param path_name - the name of the file, or sound resource identified this
   * way
   *
   * @return true if the sound resource is available, false otherwise.
   */
  virtual bool checkFile(const std::string& path_name) override { return true; }

  /**
   * Get the channels and length values of the given sound resource.
   *
   * @param path_name - the name of the file, or sound resource identified this
   * way
   * @param channels - the channels value to be filled with the sound resource
   * number of channels
   * @param length - the length value to be filled with the sound resource
   * length in frames
   *
   */
  virtual void getParamsFile(const std::string& path_name, int& channels,
                             int& length) override {
    channels = SOUND_CHAN;
    length = SOUND_LENGTH;
  }

  /**
   * Read one sound resource and fill the 'soundfile' structure accordingly
   *
   * @param path_name - the name of the file, or sound resource identified this
   * way
   * @param part - the part number to be filled in the soundfile
   * @param offset - the offset value to be incremented with the actual sound
   * resource length in frames
   * @param max_chan - the maximum number of mono channels to fill
   *
   */
  virtual void readFile(Soundfile* soundfile, const std::string& path_name,
                        int part, int& offset, int max_chan) override {
    soundfile->fLength[part] = SOUND_LENGTH;
    soundfile->fSR[part] = SOUND_SR;
    soundfile->fOffset[part] = offset;

    // Audio frames have to be written for each chan
    if (soundfile->fIsDouble) {
      for (int sample = 0; sample < SOUND_LENGTH; sample++) {
        for (int chan = 0; chan < SOUND_CHAN; chan++) {
          static_cast<double**>(soundfile->fBuffers)[chan][offset + sample] =
              0.f;
        }
      }
    } else {
      for (int sample = 0; sample < SOUND_LENGTH; sample++) {
        for (int chan = 0; chan < SOUND_CHAN; chan++) {
          static_cast<float**>(soundfile->fBuffers)[chan][offset + sample] =
              0.f;
        }
      }
    }

    // Update offset
    offset += SOUND_LENGTH;
  }
};

#endif
/**************************  END  MemoryReader.h **************************/
static MemoryReader gReader;
#else
/************************** BEGIN LibsndfileReader.h *********************
 FAUST Architecture File
 Copyright (C) 2003-2022 GRAME, Centre National de Creation Musicale
 ---------------------------------------------------------------------
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 EXCEPTION : As a special exception, you may create a larger work
 that contains this FAUST architecture section and distribute
 that work under terms of your choice, so long as this FAUST
 architecture section is not modified.
 ************************************************************************/

#ifndef __LibsndfileReader__
#define __LibsndfileReader__

#ifdef _SAMPLERATE
#include <samplerate.h>
#endif
#include <assert.h>
#include <sndfile.h>
#include <string.h>

#include <fstream>
#include <iostream>

/*
// Deactivated for now, since the macOS remote cross-compiler fails with this
code. #if __has_include(<filesystem>) && __cplusplus >= 201703L #define
HAS_FILESYSTEM #include <filesystem> namespace fs = std::filesystem; #elif
__has_include(<experimental/filesystem>) && __cplusplus >= 201103L #define
HAS_FILESYSTEM #include <experimental/filesystem> namespace fs =
std::experimental::filesystem; #endif
*/

struct VFLibsndfile {

#define SIGNED_SIZEOF(x) ((int)sizeof(x))

  unsigned char* fBuffer;
  size_t fLength;
  size_t fOffset;
  SF_VIRTUAL_IO fVIO;

  VFLibsndfile(unsigned char* buffer, size_t length)
      : fBuffer(buffer), fLength(length), fOffset(0) {
    fVIO.get_filelen = vfget_filelen;
    fVIO.seek = vfseek;
    fVIO.read = vfread;
    fVIO.write = vfwrite;
    fVIO.tell = vftell;
  }

  static sf_count_t vfget_filelen(void* user_data) {
    VFLibsndfile* vf = static_cast<VFLibsndfile*>(user_data);
    return vf->fLength;
  }

  static sf_count_t vfseek(sf_count_t offset, int whence, void* user_data) {
    VFLibsndfile* vf = static_cast<VFLibsndfile*>(user_data);
    switch (whence) {
      case SEEK_SET:
        vf->fOffset = offset;
        break;

      case SEEK_CUR:
        vf->fOffset = vf->fOffset + offset;
        break;

      case SEEK_END:
        vf->fOffset = vf->fLength + offset;
        break;

      default:
        break;
    };

    return vf->fOffset;
  }

  static sf_count_t vfread(void* ptr, sf_count_t count, void* user_data) {
    VFLibsndfile* vf = static_cast<VFLibsndfile*>(user_data);

    /*
     **	This will break badly for files over 2Gig in length, but
     **	is sufficient for testing.
     */
    if (vf->fOffset + count > vf->fLength) {
      count = vf->fLength - vf->fOffset;
    }

    memcpy(ptr, vf->fBuffer + vf->fOffset, count);
    vf->fOffset += count;

    return count;
  }

  static sf_count_t vfwrite(const void* ptr, sf_count_t count,
                            void* user_data) {
    VFLibsndfile* vf = static_cast<VFLibsndfile*>(user_data);

    /*
     **	This will break badly for files over 2Gig in length, but
     **	is sufficient for testing.
     */
    if (vf->fOffset >= SIGNED_SIZEOF(vf->fBuffer)) {
      return 0;
    }

    if (vf->fOffset + count > SIGNED_SIZEOF(vf->fBuffer)) {
      count = sizeof(vf->fBuffer) - vf->fOffset;
    }

    memcpy(vf->fBuffer + vf->fOffset, ptr, (size_t)count);
    vf->fOffset += count;

    if (vf->fOffset > vf->fLength) {
      vf->fLength = vf->fOffset;
    }

    return count;
  }

  static sf_count_t vftell(void* user_data) {
    VFLibsndfile* vf = static_cast<VFLibsndfile*>(user_data);
    return vf->fOffset;
  }
};

struct LibsndfileReader : public SoundfileReader {
  LibsndfileReader() {}

  typedef sf_count_t (*sample_read)(SNDFILE* sndfile, void* buffer,
                                    sf_count_t frames);

  // Check file
  bool checkFile(const std::string& path_name) override {
    /*
     // Better since it supports Unicode characters.
     #ifdef HAS_FILESYSTEM
     if (!fs::exists(path_name)) {
        std::cerr << "FILE NOT FOUND\n";
        return false;
     }
     #endif
     */

    std::ofstream ofs;
    ofs.open(path_name, std::ios_base::in);
    if (!ofs.is_open()) {
      return false;
    }

    SF_INFO snd_info;
    snd_info.format = 0;
    SNDFILE* snd_file = sf_open(path_name.c_str(), SFM_READ, &snd_info);
    return checkFileAux(snd_file, path_name);
  }

  bool checkFile(unsigned char* buffer, size_t length) override {
    SF_INFO snd_info;
    snd_info.format = 0;
    VFLibsndfile vio(buffer, length);
    SNDFILE* snd_file = sf_open_virtual(&vio.fVIO, SFM_READ, &snd_info, &vio);
    return checkFileAux(snd_file, "virtual file");
  }

  bool checkFileAux(SNDFILE* snd_file, const std::string& path_name) {
    if (snd_file) {
      sf_close(snd_file);
      return true;
    } else {
      std::cerr << "ERROR : cannot open '" << path_name << "' ("
                << sf_strerror(NULL) << ")" << std::endl;
      return false;
    }
  }

  // Open the file and returns its length and channels
  void getParamsFile(const std::string& path_name, int& channels,
                     int& length) override {
    SF_INFO snd_info;
    snd_info.format = 0;
    SNDFILE* snd_file = sf_open(path_name.c_str(), SFM_READ, &snd_info);
    getParamsFileAux(snd_file, snd_info, channels, length);
  }

  void getParamsFile(unsigned char* buffer, size_t size, int& channels,
                     int& length) override {
    SF_INFO snd_info;
    snd_info.format = 0;
    VFLibsndfile vio(buffer, size);
    SNDFILE* snd_file = sf_open_virtual(&vio.fVIO, SFM_READ, &snd_info, &vio);
    getParamsFileAux(snd_file, snd_info, channels, length);
  }

  void getParamsFileAux(SNDFILE* snd_file, const SF_INFO& snd_info,
                        int& channels, int& length) {
    assert(snd_file);
    channels = int(snd_info.channels);
#ifdef _SAMPLERATE
    length = (isResampling(snd_info.samplerate))
                 ? ((double(snd_info.frames) * double(fDriverSR) /
                     double(snd_info.samplerate)) +
                    BUFFER_SIZE)
                 : int(snd_info.frames);
#else
    length = int(snd_info.frames);
#endif
    sf_close(snd_file);
  }

  // Read the file
  void readFile(Soundfile* soundfile, const std::string& path_name, int part,
                int& offset, int max_chan) override {
    SF_INFO snd_info;
    snd_info.format = 0;
    SNDFILE* snd_file = sf_open(path_name.c_str(), SFM_READ, &snd_info);
    readFileAux(soundfile, snd_file, snd_info, part, offset, max_chan);
  }

  void readFile(Soundfile* soundfile, unsigned char* buffer, size_t length,
                int part, int& offset, int max_chan) override {
    SF_INFO snd_info;
    snd_info.format = 0;
    VFLibsndfile vio(buffer, length);
    SNDFILE* snd_file = sf_open_virtual(&vio.fVIO, SFM_READ, &snd_info, &vio);
    readFileAux(soundfile, snd_file, snd_info, part, offset, max_chan);
  }

  // Will be called to fill all parts from 0 to MAX_SOUNDFILE_PARTS-1
  void readFileAux(Soundfile* soundfile, SNDFILE* snd_file,
                   const SF_INFO& snd_info, int part, int& offset,
                   int max_chan) {
    assert(snd_file);
    int channels = std::min<int>(max_chan, snd_info.channels);
#ifdef _SAMPLERATE
    if (isResampling(snd_info.samplerate)) {
      soundfile->fLength[part] =
          int(double(snd_info.frames) * double(fDriverSR) /
              double(snd_info.samplerate));
      soundfile->fSR[part] = fDriverSR;
    } else {
      soundfile->fLength[part] = int(snd_info.frames);
      soundfile->fSR[part] = snd_info.samplerate;
    }
#else
    soundfile->fLength[part] = int(snd_info.frames);
    soundfile->fSR[part] = snd_info.samplerate;
#endif
    soundfile->fOffset[part] = offset;

    // Read and fill snd_info.channels number of channels
    sf_count_t nbf;

    sample_read reader;
    void* buffer_in = nullptr;
    if (soundfile->fIsDouble) {
      buffer_in = static_cast<double*>(
          alloca(BUFFER_SIZE * sizeof(double) * snd_info.channels));
      reader = reinterpret_cast<sample_read>(sf_readf_double);
    } else {
      buffer_in = static_cast<float*>(
          alloca(BUFFER_SIZE * sizeof(float) * snd_info.channels));
      reader = reinterpret_cast<sample_read>(sf_readf_float);
    }

#ifdef _SAMPLERATE
    // Resampling
    SRC_STATE* resampler = nullptr;
    float* src_buffer_out = nullptr;
    float* src_buffer_in = nullptr;
    void* buffer_out = nullptr;
    if (isResampling(snd_info.samplerate)) {
      int error;
      resampler = src_new(SRC_SINC_FASTEST, channels, &error);
      if (error != 0) {
        std::cerr << "ERROR : src_new " << src_strerror(error) << std::endl;
        throw -1;
      }
      if (soundfile->fIsDouble) {
        // Additional buffers for SRC resampling
        src_buffer_in = static_cast<float*>(
            alloca(BUFFER_SIZE * sizeof(float) * snd_info.channels));
        src_buffer_out = static_cast<float*>(
            alloca(BUFFER_SIZE * sizeof(float) * snd_info.channels));
        buffer_out = static_cast<double*>(
            alloca(BUFFER_SIZE * sizeof(double) * snd_info.channels));
      } else {
        buffer_out = static_cast<float*>(
            alloca(BUFFER_SIZE * sizeof(float) * snd_info.channels));
      }
    }
#endif

    do {
      nbf = reader(snd_file, buffer_in, BUFFER_SIZE);
#ifdef _SAMPLERATE
      // Resampling
      if (isResampling(snd_info.samplerate)) {
        int in_offset = 0;
        SRC_DATA src_data;
        src_data.src_ratio = double(fDriverSR) / double(snd_info.samplerate);
        if (soundfile->fIsDouble) {
          for (int frame = 0; frame < (BUFFER_SIZE * snd_info.channels);
               frame++) {
            src_buffer_in[frame] = float(static_cast<float*>(buffer_in)[frame]);
          }
        }
        do {
          if (soundfile->fIsDouble) {
            src_data.data_in = src_buffer_in;
            src_data.data_out = src_buffer_out;
          } else {
            src_data.data_in = static_cast<const float*>(buffer_in);
            src_data.data_out = static_cast<float*>(buffer_out);
          }
          src_data.input_frames = nbf - in_offset;
          src_data.output_frames = BUFFER_SIZE;
          src_data.end_of_input = (nbf < BUFFER_SIZE);
          int res = src_process(resampler, &src_data);
          if (res != 0) {
            std::cerr << "ERROR : src_process " << src_strerror(res)
                      << std::endl;
            throw -1;
          }
          if (soundfile->fIsDouble) {
            for (int frame = 0; frame < (BUFFER_SIZE * snd_info.channels);
                 frame++) {
              static_cast<double*>(buffer_out)[frame] =
                  double(src_buffer_out[frame]);
            }
          }
          soundfile->copyToOut(src_data.output_frames_gen, channels,
                               snd_info.channels, offset, buffer_out);
          in_offset += src_data.input_frames_used;
          // Update offset
          offset += src_data.output_frames_gen;
        } while (in_offset < nbf);
      } else {
        soundfile->copyToOut(nbf, channels, snd_info.channels, offset,
                             buffer_in);
        // Update offset
        offset += nbf;
      }
#else
      soundfile->copyToOut(nbf, channels, snd_info.channels, offset, buffer_in);
      // Update offset
      offset += nbf;
#endif
    } while (nbf == BUFFER_SIZE);

    sf_close(snd_file);
#ifdef _SAMPLERATE
    if (resampler) src_delete(resampler);
#endif
  }
};

#endif
/**************************  END  LibsndfileReader.h **************************/
static LibsndfileReader gReader;
#endif

// To be used by DSP code if no SoundUI is used
static std::vector<std::string> gPathNameList;
static Soundfile* defaultsound = nullptr;

class SoundUI : public SoundUIInterface {
 protected:
  // The soundfile directories
  Soundfile::Directories fSoundfileDir;
  // Map to share loaded soundfiles
  std::map<std::string, std::shared_ptr<Soundfile>> fSoundfileMap;
  // The soundfile reader
  std::shared_ptr<SoundfileReader> fSoundReader;
  bool fIsDouble;

 public:
  /**
   * Create a soundfile loader which will typically use a concrete
   * SoundfileReader like LibsndfileReader or JuceReader to load soundfiles.
   *
   * @param sound_directory - the base directory to look for files, which paths
   * will be relative to this one
   * @param sample_rate - the audio driver SR which may be different from the
   * file SR, to possibly resample files
   * @param reader - an alternative soundfile reader
   * @param is_double - whether Faust code has been compiled in -double mode and
   * soundfile buffers have to be in double
   *
   * @return the soundfile loader.
   */
  SoundUI(const std::string& sound_directory = "", int sample_rate = -1,
          SoundfileReader* reader = nullptr, bool is_double = false) {
    fSoundfileDir.push_back(sound_directory);
    fSoundReader = (reader) ? std::shared_ptr<SoundfileReader>(reader)
                            // the static gReader should not be deleted, so use
                            // an empty destructor
                            : std::shared_ptr<SoundfileReader>(
                                  std::shared_ptr<SoundfileReader>{}, &gReader);
    fSoundReader->setSampleRate(sample_rate);
    fIsDouble = is_double;
    if (!defaultsound)
      defaultsound =
          gReader.createSoundfile(gPathNameList, MAX_CHAN, is_double);
  }

  /**
   * Create a soundfile loader which will typically use a concrete
   * SoundfileReader like LibsndfileReader or JuceReader to load soundfiles.
   *
   * @param sound_directories - a vector of base directories to look for files,
   * which paths will be relative to these ones
   * @param sample_rate - the audio driver SR which may be different from the
   * file SR, to possibly resample files
   * @param reader - an alternative soundfile reader
   * @param is_double - whether Faust code has been compiled in -double mode and
   * soundfile buffers have to be in double
   *
   * @return the soundfile loader.
   */
  SoundUI(const Soundfile::Directories& sound_directories, int sample_rate = -1,
          SoundfileReader* reader = nullptr, bool is_double = false)
      : fSoundfileDir(sound_directories) {
    fSoundReader = (reader) ? std::shared_ptr<SoundfileReader>(reader)
                            // the static gReader should not be deleted, so use
                            // an empty destructor
                            : std::shared_ptr<SoundfileReader>(
                                  std::shared_ptr<SoundfileReader>{}, &gReader);
    fSoundReader->setSampleRate(sample_rate);
    fIsDouble = is_double;
    if (!defaultsound)
      defaultsound =
          gReader.createSoundfile(gPathNameList, MAX_CHAN, is_double);
  }

  virtual ~SoundUI() {}

  // -- soundfiles
  virtual void addSoundfile(const char* label, const char* url,
                            Soundfile** sf_zone) {
    const char* saved_url = url;  // 'url' is consumed by parseMenuList2
    std::vector<std::string> file_name_list;

    bool menu = parseMenuList2(url, file_name_list, true);
    // If not a list, we have as single file
    if (!menu) {
      file_name_list.push_back(saved_url);
    }

    // Parse the possible list
    std::string saved_url_real =
        std::string(saved_url) + "_" +
        std::to_string(fIsDouble);  // fIsDouble is used in the key
    if (fSoundfileMap.find(saved_url_real) == fSoundfileMap.end()) {
      // Check all files and get their complete path
      std::vector<std::string> path_name_list =
          fSoundReader->checkFiles(fSoundfileDir, file_name_list);
      // Read them and create the Soundfile
      Soundfile* sound_file =
          fSoundReader->createSoundfile(path_name_list, MAX_CHAN, fIsDouble);
      if (sound_file) {
        fSoundfileMap[saved_url_real] = std::shared_ptr<Soundfile>(sound_file);
      } else {
        // If failure, use 'defaultsound'
        std::cerr << "addSoundfile : soundfile for " << saved_url
                  << " cannot be created !" << std::endl;
        *sf_zone = defaultsound;
        return;
      }
    }

    // Get the soundfile pointer
    *sf_zone = fSoundfileMap[saved_url_real].get();
  }

  /**
   * An OS dependant function to get the path of the running executable or
   * plugin. This will typically be used when creating a SoundUI soundfile
   * loader, like new SoundUI(SoundUI::getBinaryPath());
   *
   * @return the running executable or plugin path.
   */
  static std::string getBinaryPath() {
    std::string bundle_path_str;
#if defined(__APPLE__) && !defined(__VCVRACK__) && !defined(JUCE_32BIT) && \
    !defined(JUCE_64BIT)
    CFURLRef bundle_ref = CFBundleCopyBundleURL(CFBundleGetMainBundle());
    if (!bundle_ref) {
      std::cerr << "getBinaryPath CFBundleCopyBundleURL error\n";
      return "";
    }

    UInt8 bundle_path[1024];
    if (CFURLGetFileSystemRepresentation(bundle_ref, true, bundle_path, 1024)) {
      bundle_path_str = std::string((char*)bundle_path);
    } else {
      std::cerr << "getBinaryPath CFURLGetFileSystemRepresentation error\n";
    }
#endif
#ifdef ANDROID_DRIVER
    bundle_path_str = "/data/data/__CURRENT_ANDROID_PACKAGE__/files";
#endif
    return bundle_path_str;
  }

  /**
   * An OS dependant function to get the path of the running executable or
   * plugin. This will typically be used when creating a SoundUI soundfile
   * loader, like new SoundUI(SoundUI::getBinaryPathFrom());
   *
   * @param path - entry point to start getting the path of the running
   * executable or plugin.
   *
   * @return the running executable or plugin path.
   */
  static std::string getBinaryPathFrom(const std::string& path) {
    std::string bundle_path_str;
#if defined(__APPLE__) && !defined(__VCVRACK__) && !defined(JUCE_32BIT) && \
    !defined(JUCE_64BIT)
    CFBundleRef bundle =
        CFBundleGetBundleWithIdentifier(CFStringCreateWithCString(
            kCFAllocatorDefault, path.c_str(), CFStringGetSystemEncoding()));
    if (!bundle) {
      std::cerr << "getBinaryPathFrom CFBundleGetBundleWithIdentifier error '"
                << path << "'" << std::endl;
      return "";
    }

    CFURLRef bundle_ref = CFBundleCopyBundleURL(bundle);
    if (!bundle_ref) {
      std::cerr << "getBinaryPathFrom CFBundleCopyBundleURL error\n";
      return "";
    }

    UInt8 bundle_path[1024];
    if (CFURLGetFileSystemRepresentation(bundle_ref, true, bundle_path, 1024)) {
      bundle_path_str = std::string((char*)bundle_path);
    } else {
      std::cerr << "getBinaryPathFrom CFURLGetFileSystemRepresentation error\n";
    }
#endif
#ifdef ANDROID_DRIVER
    bundle_path_str = "/data/data/__CURRENT_ANDROID_PACKAGE__/files";
#endif
    return bundle_path_str;
  }
};

#endif
/**************************  END  SoundUI.h **************************/
#endif

std::string defaultSoundfilesDirectory() {
  char* soundfiles_dir = getenv("FAUST_SOUNDFILES");
  return std::string((soundfiles_dir) ? soundfiles_dir : "");
}

#ifdef __APPLE__
std::string defaultUserAppSupportDirectory() {
  return std::string(getenv("HOME")) +
         "/Library/Application Support/SuperCollider/Extensions";
}
std::string defaultSoundfilesDirectory1() {
  return std::string(getenv("HOME")) +
         "/Library/Application Support/SuperCollider/Extensions/FaustSounds";
}
#else
std::string defaultUserAppSupportDirectory() { return getenv("HOME"); }
std::string defaultSoundfilesDirectory1() {
  return std::string(getenv("HOME")) + "/FaustSounds";
}
#endif

using namespace std;

#if defined(__GNUC__) && __GNUC__ >= 4
#define FAUST_EXPORT __attribute__((visibility("default")))
#else
#define FAUST_EXPORT SC_API_EXPORT
#endif

#ifdef WIN32
#define STRDUP _strdup
#else
#define STRDUP strdup
#endif

//----------------------------------------------------------------------------
// Metadata
//----------------------------------------------------------------------------

class MetaData : public Meta, public std::map<std::string, std::string> {
 public:
  void declare(const char* key, const char* value) { (*this)[key] = value; }
};

//----------------------------------------------------------------------------
// Control counter
//----------------------------------------------------------------------------

class ControlCounter : public UI {
 public:
  ControlCounter() : mNumControlInputs(0), mNumControlOutputs(0) {}

  size_t getNumControls() const { return getNumControlInputs(); }
  size_t getNumControlInputs() const { return mNumControlInputs; }
  size_t getNumControlOutputs() const { return mNumControlOutputs; }

  // Layout widgets
  virtual void openTabBox(const char* label) {}
  virtual void openHorizontalBox(const char* label) {}
  virtual void openVerticalBox(const char* label) {}
  virtual void closeBox() {}

  // Active widgets
  virtual void addButton(const char* label, FAUSTFLOAT* zone) {
    addControlInput();
  }
  virtual void addCheckButton(const char* label, FAUSTFLOAT* zone) {
    addControlInput();
  }
  virtual void addVerticalSlider(const char* label, FAUSTFLOAT* zone,
                                 FAUSTFLOAT init, FAUSTFLOAT min,
                                 FAUSTFLOAT max, FAUSTFLOAT step) {
    addControlInput();
  }
  virtual void addHorizontalSlider(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT init, FAUSTFLOAT min,
                                   FAUSTFLOAT max, FAUSTFLOAT step) {
    addControlInput();
  }
  virtual void addNumEntry(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT init,
                           FAUSTFLOAT min, FAUSTFLOAT max, FAUSTFLOAT step) {
    addControlInput();
  }

  // Passive widgets
  virtual void addHorizontalBargraph(const char* label, FAUSTFLOAT* zone,
                                     FAUSTFLOAT min, FAUSTFLOAT max) {
    addControlOutput();
  }
  virtual void addVerticalBargraph(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT min, FAUSTFLOAT max) {
    addControlOutput();
  }

  virtual void addSoundfile(const char* label, const char* filename,
                            Soundfile** sf_zone) {}

 protected:
  void addControlInput() { mNumControlInputs++; }
  void addControlOutput() { mNumControlOutputs++; }

 private:
  size_t mNumControlInputs;
  size_t mNumControlOutputs;
};

//----------------------------------------------------------------------------
// UI control
//----------------------------------------------------------------------------

struct Control {
  typedef void (*UpdateFunction)(Control* self, FAUSTFLOAT value);

  UpdateFunction updateFunction;
  FAUSTFLOAT* zone;
  FAUSTFLOAT min, max;

  inline void update(FAUSTFLOAT value) { (*updateFunction)(this, value); }

  static void simpleUpdate(Control* self, FAUSTFLOAT value) {
    *self->zone = value;
  }
  static void boundedUpdate(Control* self, FAUSTFLOAT value) {
    *self->zone = sc_clip(value, self->min, self->max);
  }
};

//----------------------------------------------------------------------------
// Control allocator
//----------------------------------------------------------------------------

class ControlAllocator : public UI {
 public:
  ControlAllocator(Control* controls) : mControls(controls) {}

  // Layout widgets
  virtual void openTabBox(const char* label) {}
  virtual void openHorizontalBox(const char* label) {}
  virtual void openVerticalBox(const char* label) {}
  virtual void closeBox() {}

  // Active widgets
  virtual void addButton(const char* label, FAUSTFLOAT* zone) {
    addSimpleControl(zone);
  }
  virtual void addCheckButton(const char* label, FAUSTFLOAT* zone) {
    addSimpleControl(zone);
  }
  virtual void addVerticalSlider(const char* label, FAUSTFLOAT* zone,
                                 FAUSTFLOAT init, FAUSTFLOAT min,
                                 FAUSTFLOAT max, FAUSTFLOAT step) {
    addBoundedControl(zone, min, max, step);
  }
  virtual void addHorizontalSlider(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT init, FAUSTFLOAT min,
                                   FAUSTFLOAT max, FAUSTFLOAT step) {
    addBoundedControl(zone, min, max, step);
  }
  virtual void addNumEntry(const char* label, FAUSTFLOAT* zone, FAUSTFLOAT init,
                           FAUSTFLOAT min, FAUSTFLOAT max, FAUSTFLOAT step) {
    addBoundedControl(zone, min, max, step);
  }

  // Passive widgets
  virtual void addHorizontalBargraph(const char* label, FAUSTFLOAT* zone,
                                     FAUSTFLOAT min, FAUSTFLOAT max) {}
  virtual void addVerticalBargraph(const char* label, FAUSTFLOAT* zone,
                                   FAUSTFLOAT min, FAUSTFLOAT max) {}
  virtual void addSoundfile(const char* label, const char* filename,
                            Soundfile** sf_zone) {}

 private:
  void addControl(Control::UpdateFunction updateFunction, FAUSTFLOAT* zone,
                  FAUSTFLOAT min, FAUSTFLOAT max, FAUSTFLOAT /* step */) {
    Control* ctrl = mControls++;
    ctrl->updateFunction = updateFunction;
    ctrl->zone = zone;
    ctrl->min = min;
    ctrl->max = max;
  }
  void addSimpleControl(FAUSTFLOAT* zone) {
    addControl(Control::simpleUpdate, zone, 0.f, 0.f, 0.f);
  }
  void addBoundedControl(FAUSTFLOAT* zone, FAUSTFLOAT min, FAUSTFLOAT max,
                         FAUSTFLOAT step) {
    addControl(Control::boundedUpdate, zone, min, max, step);
  }

 private:
  Control* mControls;
};

/******************************************************************************
 *******************************************************************************

 VECTOR INTRINSICS

 *******************************************************************************
 *******************************************************************************/

/********************END ARCHITECTURE SECTION (part 1/2)****************/

/**************************BEGIN USER SECTION **************************/

#ifndef FAUSTFLOAT
#define FAUSTFLOAT float
#endif

#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#ifndef FAUSTCLASS
#define FAUSTCLASS mydsp
#endif

#ifdef __APPLE__
#define exp10f __exp10f
#define exp10 __exp10
#endif

#if defined(_WIN32)
#define RESTRICT __restrict
#else
#define RESTRICT __restrict__
#endif

class mydspSIG0 {
 private:
  int iVec6[2];
  int iRec34[2];

 public:
  int getNumInputsmydspSIG0() { return 0; }
  int getNumOutputsmydspSIG0() { return 1; }

  void instanceInitmydspSIG0(int sample_rate) {
    for (int l21 = 0; l21 < 2; l21 = l21 + 1) {
      iVec6[l21] = 0;
    }
    for (int l22 = 0; l22 < 2; l22 = l22 + 1) {
      iRec34[l22] = 0;
    }
  }

  void fillmydspSIG0(int count, float* table) {
    for (int i1 = 0; i1 < count; i1 = i1 + 1) {
      iVec6[0] = 1;
      iRec34[0] = (iVec6[1] + iRec34[1]) % 65536;
      table[i1] = std::sin(9.58738e-05f * float(iRec34[0]));
      iVec6[1] = iVec6[0];
      iRec34[1] = iRec34[0];
    }
  }
};

static mydspSIG0* newmydspSIG0() { return (mydspSIG0*)new mydspSIG0(); }
static void deletemydspSIG0(mydspSIG0* dsp) { delete dsp; }

static float ftbl0mydspSIG0[65536];

class mydsp : public dsp {
 private:
  int fSampleRate;
  float fConst0;
  float fConst1;
  float fConst2;
  FAUSTFLOAT fHslider0;
  float fConst3;
  float fRec8[2];
  FAUSTFLOAT fHslider1;
  float fConst4;
  float fRec12[2];
  FAUSTFLOAT fHslider2;
  float fRec15[2];
  FAUSTFLOAT fHslider3;
  float fRec20[2];
  FAUSTFLOAT fHslider4;
  float fRec24[2];
  FAUSTFLOAT fHslider5;
  float fRec25[2];
  FAUSTFLOAT fHslider6;
  float fRec27[2];
  int IOTA0;
  float fVec0[131072];
  FAUSTFLOAT fHslider7;
  float fConst5;
  float fRec28[2];
  float fRec26[2];
  float fRec23[2];
  float fVec1[1024];
  int iConst6;
  float fRec21[2];
  float fVec2[1024];
  int iConst7;
  float fRec18[2];
  float fVec3[4096];
  int iConst8;
  float fRec16[2];
  float fVec4[2048];
  int iConst9;
  float fRec13[2];
  int iConst10;
  FAUSTFLOAT fHslider8;
  float fRec29[2];
  float fVec5[131072];
  FAUSTFLOAT fHslider9;
  float fRec36[2];
  float fConst11;
  float fRec35[2];
  FAUSTFLOAT fHslider10;
  float fRec37[2];
  float fConst12;
  float fRec30[2];
  float fRec31[2];
  int iRec32[2];
  int iRec33[2];
  float fRec10[2];
  float fVec7[32768];
  int iConst13;
  float fRec9[2];
  float fVec8[32768];
  int iConst14;
  float fRec6[2];
  float fRec0[32768];
  float fRec1[16384];
  float fRec2[32768];
  float fVec9[131072];
  float fRec52[2];
  float fRec51[2];
  float fVec10[1024];
  int iConst15;
  float fRec49[2];
  float fVec11[1024];
  int iConst16;
  float fRec47[2];
  float fVec12[4096];
  int iConst17;
  float fRec45[2];
  float fVec13[2048];
  int iConst18;
  float fRec43[2];
  int iConst19;
  float fVec14[131072];
  float fRec53[2];
  float fRec54[2];
  int iRec55[2];
  int iRec56[2];
  float fRec41[2];
  float fVec15[32768];
  int iConst20;
  float fRec40[2];
  float fVec16[16384];
  int iConst21;
  float fRec38[2];
  float fRec3[32768];
  float fRec4[8192];
  float fRec5[32768];
  int iConst22;
  int iConst23;
  int iConst24;
  int iConst25;
  int iConst26;
  int iConst27;
  int iConst28;
  int iConst29;
  int iConst30;
  int iConst31;
  int iConst32;
  int iConst33;
  int iConst34;
  int iConst35;

 public:
  mydsp() {}

  void metadata(Meta* m) {
    m->declare("author", "Jean Pierre Cimalando");
    m->declare("basics.lib/name", "Faust Basic Element Library");
    m->declare("basics.lib/tabulateNd",
               "Copyright (C) 2023 Bart Brouns <bart@magnetophon.nl>");
    m->declare("basics.lib/version", "1.12.0");
    m->declare("compile_options",
               "-a supercollider.cpp -lang cpp -i -ct 1 -es 1 -mcd 16 -mdd "
               "1024 -mdy 33 -single -ftz 0");
    m->declare("delays.lib/name", "Faust Delay Library");
    m->declare("delays.lib/version", "1.1.0");
    m->declare("filename", "fverb2.dsp");
    m->declare("filters.lib/allpass_comb:author", "Julius O. Smith III");
    m->declare("filters.lib/allpass_comb:copyright",
               "Copyright (C) 2003-2019 by Julius O. Smith III "
               "<jos@ccrma.stanford.edu>");
    m->declare("filters.lib/allpass_comb:license", "MIT-style STK-4.3 license");
    m->declare("filters.lib/fir:author", "Julius O. Smith III");
    m->declare("filters.lib/fir:copyright",
               "Copyright (C) 2003-2019 by Julius O. Smith III "
               "<jos@ccrma.stanford.edu>");
    m->declare("filters.lib/fir:license", "MIT-style STK-4.3 license");
    m->declare("filters.lib/iir:author", "Julius O. Smith III");
    m->declare("filters.lib/iir:copyright",
               "Copyright (C) 2003-2019 by Julius O. Smith III "
               "<jos@ccrma.stanford.edu>");
    m->declare("filters.lib/iir:license", "MIT-style STK-4.3 license");
    m->declare("filters.lib/lowpass0_highpass1",
               "Copyright (C) 2003-2019 by Julius O. Smith III "
               "<jos@ccrma.stanford.edu>");
    m->declare("filters.lib/name", "Faust Filters Library");
    m->declare("filters.lib/version", "1.3.0");
    m->declare("license", "BSD-2-Clause");
    m->declare("maths.lib/author", "GRAME");
    m->declare("maths.lib/copyright", "GRAME");
    m->declare("maths.lib/license", "LGPL with exception");
    m->declare("maths.lib/name", "Faust Math Library");
    m->declare("maths.lib/version", "2.7.0");
    m->declare("name", "fverb2");
    m->declare("oscillators.lib/name", "Faust Oscillator Library");
    m->declare("oscillators.lib/version", "1.5.0");
    m->declare("platform.lib/name", "Generic Platform Library");
    m->declare("platform.lib/version", "1.3.0");
    m->declare("signals.lib/name", "Faust Signal Routing Library");
    m->declare("signals.lib/version", "1.5.0");
    m->declare("version", "0.5");
  }

  virtual int getNumInputs() { return 2; }
  virtual int getNumOutputs() { return 2; }

  static void classInit(int sample_rate) {
    mydspSIG0* sig0 = newmydspSIG0();
    sig0->instanceInitmydspSIG0(sample_rate);
    sig0->fillmydspSIG0(65536, ftbl0mydspSIG0);
    deletemydspSIG0(sig0);
  }

  virtual void instanceConstants(int sample_rate) {
    fSampleRate = sample_rate;
    fConst0 =
        std::min<float>(1.92e+05f, std::max<float>(1.0f, float(fSampleRate)));
    fConst1 = 44.1f / fConst0;
    fConst2 = 1.0f - fConst1;
    fConst3 = 6.2831855f / fConst0;
    fConst4 = 0.441f / fConst0;
    fConst5 = 0.0441f / fConst0;
    iConst6 = std::min<int>(
        65536, std::max<int>(0, int(0.0046282047f * fConst0) + -1));
    iConst7 = std::min<int>(
        65536, std::max<int>(0, int(0.0037031686f * fConst0) + -1));
    iConst8 = std::min<int>(65536,
                            std::max<int>(0, int(0.013116831f * fConst0) + -1));
    iConst9 = std::min<int>(65536,
                            std::max<int>(0, int(0.009028259f * fConst0) + -1));
    iConst10 =
        std::min<int>(65536, std::max<int>(0, int(0.10628003f * fConst0))) + 1;
    fConst11 = 1.0f / fConst0;
    fConst12 = 1.0f / float(int(0.01f * fConst0));
    iConst13 =
        std::min<int>(65536, std::max<int>(0, int(0.14169551f * fConst0)));
    iConst14 =
        std::min<int>(65536, std::max<int>(0, int(0.08924431f * fConst0) + -1));
    iConst15 = std::min<int>(
        65536, std::max<int>(0, int(0.0049144854f * fConst0) + -1));
    iConst16 =
        std::min<int>(65536, std::max<int>(0, int(0.00348745f * fConst0) + -1));
    iConst17 = std::min<int>(
        65536, std::max<int>(0, int(0.012352743f * fConst0) + -1));
    iConst18 = std::min<int>(
        65536, std::max<int>(0, int(0.009586708f * fConst0) + -1));
    iConst19 =
        std::min<int>(65536, std::max<int>(0, int(0.1249958f * fConst0))) + 1;
    iConst20 =
        std::min<int>(65536, std::max<int>(0, int(0.14962535f * fConst0)));
    iConst21 =
        std::min<int>(65536, std::max<int>(0, int(0.06048184f * fConst0) + -1));
    iConst22 =
        std::min<int>(65536, std::max<int>(0, int(0.03581869f * fConst0)));
    iConst23 =
        std::min<int>(65536, std::max<int>(0, int(0.006283391f * fConst0)));
    iConst24 =
        std::min<int>(65536, std::max<int>(0, int(0.06686603f * fConst0)));
    iConst25 =
        std::min<int>(65536, std::max<int>(0, int(0.06427875f * fConst0)));
    iConst26 =
        std::min<int>(65536, std::max<int>(0, int(0.06706764f * fConst0)));
    iConst27 =
        std::min<int>(65536, std::max<int>(0, int(0.09992944f * fConst0)));
    iConst28 =
        std::min<int>(65536, std::max<int>(0, int(0.008937872f * fConst0)));
    iConst29 =
        std::min<int>(65536, std::max<int>(0, int(0.0040657236f * fConst0)));
    iConst30 =
        std::min<int>(65536, std::max<int>(0, int(0.011256342f * fConst0)));
    iConst31 =
        std::min<int>(65536, std::max<int>(0, int(0.070931755f * fConst0)));
    iConst32 =
        std::min<int>(65536, std::max<int>(0, int(0.041262053f * fConst0)));
    iConst33 =
        std::min<int>(65536, std::max<int>(0, int(0.08981553f * fConst0)));
    iConst34 =
        std::min<int>(65536, std::max<int>(0, int(0.121870905f * fConst0)));
    iConst35 =
        std::min<int>(65536, std::max<int>(0, int(0.01186116f * fConst0)));
  }

  virtual void instanceResetUserInterface() {
    fHslider0 = FAUSTFLOAT(5.5e+03f);
    fHslider1 = FAUSTFLOAT(7e+01f);
    fHslider2 = FAUSTFLOAT(62.5f);
    fHslider3 = FAUSTFLOAT(75.0f);
    fHslider4 = FAUSTFLOAT(1e+02f);
    fHslider5 = FAUSTFLOAT(1e+04f);
    fHslider6 = FAUSTFLOAT(1e+02f);
    fHslider7 = FAUSTFLOAT(0.0f);
    fHslider8 = FAUSTFLOAT(5e+01f);
    fHslider9 = FAUSTFLOAT(1.0f);
    fHslider10 = FAUSTFLOAT(0.5f);
  }

  virtual void instanceClear() {
    for (int l0 = 0; l0 < 2; l0 = l0 + 1) {
      fRec8[l0] = 0.0f;
    }
    for (int l1 = 0; l1 < 2; l1 = l1 + 1) {
      fRec12[l1] = 0.0f;
    }
    for (int l2 = 0; l2 < 2; l2 = l2 + 1) {
      fRec15[l2] = 0.0f;
    }
    for (int l3 = 0; l3 < 2; l3 = l3 + 1) {
      fRec20[l3] = 0.0f;
    }
    for (int l4 = 0; l4 < 2; l4 = l4 + 1) {
      fRec24[l4] = 0.0f;
    }
    for (int l5 = 0; l5 < 2; l5 = l5 + 1) {
      fRec25[l5] = 0.0f;
    }
    for (int l6 = 0; l6 < 2; l6 = l6 + 1) {
      fRec27[l6] = 0.0f;
    }
    IOTA0 = 0;
    for (int l7 = 0; l7 < 131072; l7 = l7 + 1) {
      fVec0[l7] = 0.0f;
    }
    for (int l8 = 0; l8 < 2; l8 = l8 + 1) {
      fRec28[l8] = 0.0f;
    }
    for (int l9 = 0; l9 < 2; l9 = l9 + 1) {
      fRec26[l9] = 0.0f;
    }
    for (int l10 = 0; l10 < 2; l10 = l10 + 1) {
      fRec23[l10] = 0.0f;
    }
    for (int l11 = 0; l11 < 1024; l11 = l11 + 1) {
      fVec1[l11] = 0.0f;
    }
    for (int l12 = 0; l12 < 2; l12 = l12 + 1) {
      fRec21[l12] = 0.0f;
    }
    for (int l13 = 0; l13 < 1024; l13 = l13 + 1) {
      fVec2[l13] = 0.0f;
    }
    for (int l14 = 0; l14 < 2; l14 = l14 + 1) {
      fRec18[l14] = 0.0f;
    }
    for (int l15 = 0; l15 < 4096; l15 = l15 + 1) {
      fVec3[l15] = 0.0f;
    }
    for (int l16 = 0; l16 < 2; l16 = l16 + 1) {
      fRec16[l16] = 0.0f;
    }
    for (int l17 = 0; l17 < 2048; l17 = l17 + 1) {
      fVec4[l17] = 0.0f;
    }
    for (int l18 = 0; l18 < 2; l18 = l18 + 1) {
      fRec13[l18] = 0.0f;
    }
    for (int l19 = 0; l19 < 2; l19 = l19 + 1) {
      fRec29[l19] = 0.0f;
    }
    for (int l20 = 0; l20 < 131072; l20 = l20 + 1) {
      fVec5[l20] = 0.0f;
    }
    for (int l23 = 0; l23 < 2; l23 = l23 + 1) {
      fRec36[l23] = 0.0f;
    }
    for (int l24 = 0; l24 < 2; l24 = l24 + 1) {
      fRec35[l24] = 0.0f;
    }
    for (int l25 = 0; l25 < 2; l25 = l25 + 1) {
      fRec37[l25] = 0.0f;
    }
    for (int l26 = 0; l26 < 2; l26 = l26 + 1) {
      fRec30[l26] = 0.0f;
    }
    for (int l27 = 0; l27 < 2; l27 = l27 + 1) {
      fRec31[l27] = 0.0f;
    }
    for (int l28 = 0; l28 < 2; l28 = l28 + 1) {
      iRec32[l28] = 0;
    }
    for (int l29 = 0; l29 < 2; l29 = l29 + 1) {
      iRec33[l29] = 0;
    }
    for (int l30 = 0; l30 < 2; l30 = l30 + 1) {
      fRec10[l30] = 0.0f;
    }
    for (int l31 = 0; l31 < 32768; l31 = l31 + 1) {
      fVec7[l31] = 0.0f;
    }
    for (int l32 = 0; l32 < 2; l32 = l32 + 1) {
      fRec9[l32] = 0.0f;
    }
    for (int l33 = 0; l33 < 32768; l33 = l33 + 1) {
      fVec8[l33] = 0.0f;
    }
    for (int l34 = 0; l34 < 2; l34 = l34 + 1) {
      fRec6[l34] = 0.0f;
    }
    for (int l35 = 0; l35 < 32768; l35 = l35 + 1) {
      fRec0[l35] = 0.0f;
    }
    for (int l36 = 0; l36 < 16384; l36 = l36 + 1) {
      fRec1[l36] = 0.0f;
    }
    for (int l37 = 0; l37 < 32768; l37 = l37 + 1) {
      fRec2[l37] = 0.0f;
    }
    for (int l38 = 0; l38 < 131072; l38 = l38 + 1) {
      fVec9[l38] = 0.0f;
    }
    for (int l39 = 0; l39 < 2; l39 = l39 + 1) {
      fRec52[l39] = 0.0f;
    }
    for (int l40 = 0; l40 < 2; l40 = l40 + 1) {
      fRec51[l40] = 0.0f;
    }
    for (int l41 = 0; l41 < 1024; l41 = l41 + 1) {
      fVec10[l41] = 0.0f;
    }
    for (int l42 = 0; l42 < 2; l42 = l42 + 1) {
      fRec49[l42] = 0.0f;
    }
    for (int l43 = 0; l43 < 1024; l43 = l43 + 1) {
      fVec11[l43] = 0.0f;
    }
    for (int l44 = 0; l44 < 2; l44 = l44 + 1) {
      fRec47[l44] = 0.0f;
    }
    for (int l45 = 0; l45 < 4096; l45 = l45 + 1) {
      fVec12[l45] = 0.0f;
    }
    for (int l46 = 0; l46 < 2; l46 = l46 + 1) {
      fRec45[l46] = 0.0f;
    }
    for (int l47 = 0; l47 < 2048; l47 = l47 + 1) {
      fVec13[l47] = 0.0f;
    }
    for (int l48 = 0; l48 < 2; l48 = l48 + 1) {
      fRec43[l48] = 0.0f;
    }
    for (int l49 = 0; l49 < 131072; l49 = l49 + 1) {
      fVec14[l49] = 0.0f;
    }
    for (int l50 = 0; l50 < 2; l50 = l50 + 1) {
      fRec53[l50] = 0.0f;
    }
    for (int l51 = 0; l51 < 2; l51 = l51 + 1) {
      fRec54[l51] = 0.0f;
    }
    for (int l52 = 0; l52 < 2; l52 = l52 + 1) {
      iRec55[l52] = 0;
    }
    for (int l53 = 0; l53 < 2; l53 = l53 + 1) {
      iRec56[l53] = 0;
    }
    for (int l54 = 0; l54 < 2; l54 = l54 + 1) {
      fRec41[l54] = 0.0f;
    }
    for (int l55 = 0; l55 < 32768; l55 = l55 + 1) {
      fVec15[l55] = 0.0f;
    }
    for (int l56 = 0; l56 < 2; l56 = l56 + 1) {
      fRec40[l56] = 0.0f;
    }
    for (int l57 = 0; l57 < 16384; l57 = l57 + 1) {
      fVec16[l57] = 0.0f;
    }
    for (int l58 = 0; l58 < 2; l58 = l58 + 1) {
      fRec38[l58] = 0.0f;
    }
    for (int l59 = 0; l59 < 32768; l59 = l59 + 1) {
      fRec3[l59] = 0.0f;
    }
    for (int l60 = 0; l60 < 8192; l60 = l60 + 1) {
      fRec4[l60] = 0.0f;
    }
    for (int l61 = 0; l61 < 32768; l61 = l61 + 1) {
      fRec5[l61] = 0.0f;
    }
  }

  virtual void init(int sample_rate) {
    classInit(sample_rate);
    instanceInit(sample_rate);
  }

  virtual void instanceInit(int sample_rate) {
    instanceConstants(sample_rate);
    instanceResetUserInterface();
    instanceClear();
  }

  virtual mydsp* clone() { return new mydsp(); }

  virtual int getSampleRate() { return fSampleRate; }

  virtual void buildUserInterface(UI* ui_interface) {
    ui_interface->openVerticalBox("fverb2");
    ui_interface->declare(&fHslider7, "01", "");
    ui_interface->declare(&fHslider7, "symbol", "predelay");
    ui_interface->declare(&fHslider7, "unit", "ms");
    ui_interface->addHorizontalSlider("Predelay", &fHslider7, FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(0.0f), FAUSTFLOAT(3e+02f),
                                      FAUSTFLOAT(1.0f));
    ui_interface->declare(&fHslider6, "02", "");
    ui_interface->declare(&fHslider6, "symbol", "input");
    ui_interface->declare(&fHslider6, "unit", "%");
    ui_interface->addHorizontalSlider("Input amount", &fHslider6,
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider5, "03", "");
    ui_interface->declare(&fHslider5, "scale", "log");
    ui_interface->declare(&fHslider5, "symbol", "input_lowpass");
    ui_interface->declare(&fHslider5, "unit", "Hz");
    ui_interface->addHorizontalSlider("Input low-pass cutoff", &fHslider5,
                                      FAUSTFLOAT(1e+04f), FAUSTFLOAT(1.0f),
                                      FAUSTFLOAT(2e+04f), FAUSTFLOAT(1.0f));
    ui_interface->declare(&fHslider4, "04", "");
    ui_interface->declare(&fHslider4, "scale", "log");
    ui_interface->declare(&fHslider4, "symbol", "input_highpass");
    ui_interface->declare(&fHslider4, "unit", "Hz");
    ui_interface->addHorizontalSlider("Input high-pass cutoff", &fHslider4,
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(1.0f),
                                      FAUSTFLOAT(1e+03f), FAUSTFLOAT(1.0f));
    ui_interface->declare(&fHslider3, "05", "");
    ui_interface->declare(&fHslider3, "symbol", "input_diffusion_1");
    ui_interface->declare(&fHslider3, "unit", "%");
    ui_interface->addHorizontalSlider("Input diffusion 1", &fHslider3,
                                      FAUSTFLOAT(75.0f), FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider2, "06", "");
    ui_interface->declare(&fHslider2, "symbol", "input_diffusion_2");
    ui_interface->declare(&fHslider2, "unit", "%");
    ui_interface->addHorizontalSlider("Input diffusion 2", &fHslider2,
                                      FAUSTFLOAT(62.5f), FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider1, "07", "");
    ui_interface->declare(&fHslider1, "symbol", "tail_density");
    ui_interface->declare(&fHslider1, "unit", "%");
    ui_interface->addHorizontalSlider("Tail density", &fHslider1,
                                      FAUSTFLOAT(7e+01f), FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(1e+02f), FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider8, "08", "");
    ui_interface->declare(&fHslider8, "symbol", "decay");
    ui_interface->declare(&fHslider8, "unit", "%");
    ui_interface->addHorizontalSlider("Decay", &fHslider8, FAUSTFLOAT(5e+01f),
                                      FAUSTFLOAT(0.0f), FAUSTFLOAT(1e+02f),
                                      FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider0, "09", "");
    ui_interface->declare(&fHslider0, "scale", "log");
    ui_interface->declare(&fHslider0, "symbol", "damping");
    ui_interface->declare(&fHslider0, "unit", "Hz");
    ui_interface->addHorizontalSlider("Damping", &fHslider0,
                                      FAUSTFLOAT(5.5e+03f), FAUSTFLOAT(1e+01f),
                                      FAUSTFLOAT(2e+04f), FAUSTFLOAT(1.0f));
    ui_interface->declare(&fHslider9, "10", "");
    ui_interface->declare(&fHslider9, "symbol", "mod_frequency");
    ui_interface->declare(&fHslider9, "unit", "Hz");
    ui_interface->addHorizontalSlider("Modulator frequency", &fHslider9,
                                      FAUSTFLOAT(1.0f), FAUSTFLOAT(0.01f),
                                      FAUSTFLOAT(4.0f), FAUSTFLOAT(0.01f));
    ui_interface->declare(&fHslider10, "11", "");
    ui_interface->declare(&fHslider10, "symbol", "mod_depth");
    ui_interface->declare(&fHslider10, "unit", "ms");
    ui_interface->addHorizontalSlider("Modulator depth", &fHslider10,
                                      FAUSTFLOAT(0.5f), FAUSTFLOAT(0.0f),
                                      FAUSTFLOAT(1e+01f), FAUSTFLOAT(0.1f));
    ui_interface->closeBox();
  }

  virtual void compute(int count, FAUSTFLOAT** RESTRICT inputs,
                       FAUSTFLOAT** RESTRICT outputs) {
    FAUSTFLOAT* input0 = inputs[0];
    FAUSTFLOAT* input1 = inputs[1];
    FAUSTFLOAT* output0 = outputs[0];
    FAUSTFLOAT* output1 = outputs[1];
    float fSlow0 = fConst1 * std::exp(-(fConst3 * float(fHslider0)));
    float fSlow1 = fConst4 * float(fHslider1);
    float fSlow2 = fConst4 * float(fHslider2);
    float fSlow3 = fConst4 * float(fHslider3);
    float fSlow4 = fConst1 * std::exp(-(fConst3 * float(fHslider4)));
    float fSlow5 = fConst1 * std::exp(-(fConst3 * float(fHslider5)));
    float fSlow6 = fConst4 * float(fHslider6);
    float fSlow7 = fConst5 * float(fHslider7);
    float fSlow8 = fConst4 * float(fHslider8);
    float fSlow9 = fConst1 * float(fHslider9);
    float fSlow10 = fConst5 * float(fHslider10);
    for (int i0 = 0; i0 < count; i0 = i0 + 1) {
      fRec8[0] = fSlow0 + fConst2 * fRec8[1];
      float fTemp0 = 1.0f - fRec8[0];
      fRec12[0] = fSlow1 + fConst2 * fRec12[1];
      fRec15[0] = fSlow2 + fConst2 * fRec15[1];
      fRec20[0] = fSlow3 + fConst2 * fRec20[1];
      fRec24[0] = fSlow4 + fConst2 * fRec24[1];
      fRec25[0] = fSlow5 + fConst2 * fRec25[1];
      float fTemp1 = 1.0f - fRec25[0];
      fRec27[0] = fSlow6 + fConst2 * fRec27[1];
      fVec0[IOTA0 & 131071] = float(input1[i0]) * fRec27[0];
      fRec28[0] = fSlow7 + fConst2 * fRec28[1];
      int iTemp2 =
          std::min<int>(65536, std::max<int>(0, int(fConst0 * fRec28[0])));
      fRec26[0] = fVec0[(IOTA0 - iTemp2) & 131071] + fRec25[0] * fRec26[1];
      fRec23[0] = fRec26[0] * fTemp1 + fRec24[0] * fRec23[1];
      float fTemp3 = fRec24[0] + 1.0f;
      float fTemp4 =
          0.5f * fTemp3 * (fRec23[0] - fRec23[1]) - fRec20[0] * fRec21[1];
      fVec1[IOTA0 & 1023] = fTemp4;
      fRec21[0] = fVec1[(IOTA0 - iConst6) & 1023];
      float fRec22 = fRec20[0] * fTemp4;
      float fTemp5 = fRec22 + fRec21[1] - fRec20[0] * fRec18[1];
      fVec2[IOTA0 & 1023] = fTemp5;
      fRec18[0] = fVec2[(IOTA0 - iConst7) & 1023];
      float fRec19 = fRec20[0] * fTemp5;
      float fTemp6 = fRec19 + fRec18[1] - fRec15[0] * fRec16[1];
      fVec3[IOTA0 & 4095] = fTemp6;
      fRec16[0] = fVec3[(IOTA0 - iConst8) & 4095];
      float fRec17 = fRec15[0] * fTemp6;
      float fTemp7 = fRec17 + fRec16[1] - fRec15[0] * fRec13[1];
      fVec4[IOTA0 & 2047] = fTemp7;
      fRec13[0] = fVec4[(IOTA0 - iConst9) & 2047];
      float fRec14 = fRec15[0] * fTemp7;
      fRec29[0] = fSlow8 + fConst2 * fRec29[1];
      float fTemp8 = fRec13[1] + fRec29[0] * fRec3[(IOTA0 - iConst10) & 32767] +
                     fRec14 + fRec12[0] * fRec10[1];
      fVec5[IOTA0 & 131071] = fTemp8;
      fRec36[0] = fSlow9 + fConst2 * fRec36[1];
      float fTemp9 = fRec35[1] + fConst11 * fRec36[0];
      fRec35[0] = fTemp9 - float(int(fTemp9));
      fRec37[0] = fSlow10 + fConst2 * fRec37[1];
      int iTemp10 =
          int(fConst0 *
              (fRec37[0] *
                   ftbl0mydspSIG0[std::max<int>(
                       0, std::min<int>(
                              int(65536.0f *
                                  (fRec35[0] +
                                   (0.25f - float(int(fRec35[0] + 0.25f))))),
                              65535))] +
               0.030509727f)) +
          -1;
      float fTemp11 =
          ((fRec30[1] != 0.0f)
               ? (((fRec31[1] > 0.0f) & (fRec31[1] < 1.0f)) ? fRec30[1] : 0.0f)
               : (((fRec31[1] == 0.0f) & (iTemp10 != iRec32[1]))
                      ? fConst12
                      : (((fRec31[1] == 1.0f) & (iTemp10 != iRec33[1]))
                             ? -fConst12
                             : 0.0f)));
      fRec30[0] = fTemp11;
      fRec31[0] =
          std::max<float>(0.0f, std::min<float>(1.0f, fRec31[1] + fTemp11));
      iRec32[0] = (((fRec31[1] >= 1.0f) & (iRec33[1] != iTemp10)) ? iTemp10
                                                                  : iRec32[1]);
      iRec33[0] = (((fRec31[1] <= 0.0f) & (iRec32[1] != iTemp10)) ? iTemp10
                                                                  : iRec33[1]);
      float fTemp12 =
          fVec5[(IOTA0 - std::min<int>(65536, std::max<int>(0, iRec32[0]))) &
                131071];
      fRec10[0] =
          fTemp12 +
          fRec31[0] * (fVec5[(IOTA0 - std::min<int>(
                                          65536, std::max<int>(0, iRec33[0]))) &
                             131071] -
                       fTemp12);
      float fRec11 = -(fRec12[0] * fTemp8);
      float fTemp13 = fRec11 + fRec10[1];
      fVec7[IOTA0 & 32767] = fTemp13;
      fRec9[0] = fVec7[(IOTA0 - iConst13) & 32767] + fRec8[0] * fRec9[1];
      float fTemp14 =
          std::min<float>(0.5f, std::max<float>(0.25f, fRec29[0] + 0.15f));
      float fTemp15 = fTemp14 * fRec6[1] + fRec29[0] * fRec9[0] * fTemp0;
      fVec8[IOTA0 & 32767] = fTemp15;
      fRec6[0] = fVec8[(IOTA0 - iConst14) & 32767];
      float fRec7 = -(fTemp14 * fTemp15);
      fRec0[IOTA0 & 32767] = fRec7 + fRec6[1];
      fRec1[IOTA0 & 16383] = fRec9[0] * fTemp0;
      fRec2[IOTA0 & 32767] = fTemp13;
      fVec9[IOTA0 & 131071] = float(input0[i0]) * fRec27[0];
      fRec52[0] = fVec9[(IOTA0 - iTemp2) & 131071] + fRec25[0] * fRec52[1];
      fRec51[0] = fTemp1 * fRec52[0] + fRec24[0] * fRec51[1];
      float fTemp16 =
          0.5f * fTemp3 * (fRec51[0] - fRec51[1]) - fRec20[0] * fRec49[1];
      fVec10[IOTA0 & 1023] = fTemp16;
      fRec49[0] = fVec10[(IOTA0 - iConst15) & 1023];
      float fRec50 = fRec20[0] * fTemp16;
      float fTemp17 = fRec50 + fRec49[1] - fRec20[0] * fRec47[1];
      fVec11[IOTA0 & 1023] = fTemp17;
      fRec47[0] = fVec11[(IOTA0 - iConst16) & 1023];
      float fRec48 = fRec20[0] * fTemp17;
      float fTemp18 = fRec48 + fRec47[1] - fRec15[0] * fRec45[1];
      fVec12[IOTA0 & 4095] = fTemp18;
      fRec45[0] = fVec12[(IOTA0 - iConst17) & 4095];
      float fRec46 = fRec15[0] * fTemp18;
      float fTemp19 = fRec46 + fRec45[1] - fRec15[0] * fRec43[1];
      fVec13[IOTA0 & 2047] = fTemp19;
      fRec43[0] = fVec13[(IOTA0 - iConst18) & 2047];
      float fRec44 = fRec15[0] * fTemp19;
      float fTemp20 = fRec43[1] +
                      fRec29[0] * fRec0[(IOTA0 - iConst19) & 32767] + fRec44 +
                      fRec12[0] * fRec41[1];
      fVec14[IOTA0 & 131071] = fTemp20;
      int iTemp21 =
          int(fConst0 *
              (fRec37[0] *
                   ftbl0mydspSIG0[std::max<int>(
                       0, std::min<int>(int(65536.0f * fRec35[0]), 65535))] +
               0.025603978f)) +
          -1;
      float fTemp22 =
          ((fRec53[1] != 0.0f)
               ? (((fRec54[1] > 0.0f) & (fRec54[1] < 1.0f)) ? fRec53[1] : 0.0f)
               : (((fRec54[1] == 0.0f) & (iTemp21 != iRec55[1]))
                      ? fConst12
                      : (((fRec54[1] == 1.0f) & (iTemp21 != iRec56[1]))
                             ? -fConst12
                             : 0.0f)));
      fRec53[0] = fTemp22;
      fRec54[0] =
          std::max<float>(0.0f, std::min<float>(1.0f, fRec54[1] + fTemp22));
      iRec55[0] = (((fRec54[1] >= 1.0f) & (iRec56[1] != iTemp21)) ? iTemp21
                                                                  : iRec55[1]);
      iRec56[0] = (((fRec54[1] <= 0.0f) & (iRec55[1] != iTemp21)) ? iTemp21
                                                                  : iRec56[1]);
      float fTemp23 =
          fVec14[(IOTA0 - std::min<int>(65536, std::max<int>(0, iRec55[0]))) &
                 131071];
      fRec41[0] =
          fTemp23 +
          fRec54[0] *
              (fVec14[(IOTA0 -
                       std::min<int>(65536, std::max<int>(0, iRec56[0]))) &
                      131071] -
               fTemp23);
      float fRec42 = -(fRec12[0] * fTemp20);
      float fTemp24 = fRec42 + fRec41[1];
      fVec15[IOTA0 & 32767] = fTemp24;
      fRec40[0] = fVec15[(IOTA0 - iConst20) & 32767] + fRec8[0] * fRec40[1];
      float fTemp25 = fTemp14 * fRec38[1] + fRec29[0] * fTemp0 * fRec40[0];
      fVec16[IOTA0 & 16383] = fTemp25;
      fRec38[0] = fVec16[(IOTA0 - iConst21) & 16383];
      float fRec39 = -(fTemp14 * fTemp25);
      fRec3[IOTA0 & 32767] = fRec39 + fRec38[1];
      fRec4[IOTA0 & 8191] = fTemp0 * fRec40[0];
      fRec5[IOTA0 & 32767] = fTemp24;
      output0[i0] = FAUSTFLOAT(0.6f * (fRec2[(IOTA0 - iConst28) & 32767] +
                                       fRec2[(IOTA0 - iConst27) & 32767] +
                                       fRec0[(IOTA0 - iConst26) & 32767] -
                                       (fRec1[(IOTA0 - iConst25) & 16383] +
                                        fRec5[(IOTA0 - iConst24) & 32767] +
                                        fRec4[(IOTA0 - iConst23) & 8191] +
                                        fRec3[(IOTA0 - iConst22) & 32767])));
      output1[i0] = FAUSTFLOAT(0.6f * (fRec5[(IOTA0 - iConst35) & 32767] +
                                       fRec5[(IOTA0 - iConst34) & 32767] +
                                       fRec3[(IOTA0 - iConst33) & 32767] -
                                       (fRec4[(IOTA0 - iConst32) & 8191] +
                                        fRec2[(IOTA0 - iConst31) & 32767] +
                                        fRec1[(IOTA0 - iConst30) & 16383] +
                                        fRec0[(IOTA0 - iConst29) & 32767])));
      fRec8[1] = fRec8[0];
      fRec12[1] = fRec12[0];
      fRec15[1] = fRec15[0];
      fRec20[1] = fRec20[0];
      fRec24[1] = fRec24[0];
      fRec25[1] = fRec25[0];
      fRec27[1] = fRec27[0];
      IOTA0 = IOTA0 + 1;
      fRec28[1] = fRec28[0];
      fRec26[1] = fRec26[0];
      fRec23[1] = fRec23[0];
      fRec21[1] = fRec21[0];
      fRec18[1] = fRec18[0];
      fRec16[1] = fRec16[0];
      fRec13[1] = fRec13[0];
      fRec29[1] = fRec29[0];
      fRec36[1] = fRec36[0];
      fRec35[1] = fRec35[0];
      fRec37[1] = fRec37[0];
      fRec30[1] = fRec30[0];
      fRec31[1] = fRec31[0];
      iRec32[1] = iRec32[0];
      iRec33[1] = iRec33[0];
      fRec10[1] = fRec10[0];
      fRec9[1] = fRec9[0];
      fRec6[1] = fRec6[0];
      fRec52[1] = fRec52[0];
      fRec51[1] = fRec51[0];
      fRec49[1] = fRec49[0];
      fRec47[1] = fRec47[0];
      fRec45[1] = fRec45[0];
      fRec43[1] = fRec43[0];
      fRec53[1] = fRec53[0];
      fRec54[1] = fRec54[0];
      iRec55[1] = iRec55[0];
      iRec56[1] = iRec56[0];
      fRec41[1] = fRec41[0];
      fRec40[1] = fRec40[0];
      fRec38[1] = fRec38[0];
    }
  }
};

/***************************END USER SECTION ***************************/

/*******************BEGIN ARCHITECTURE SECTION (part 2/2)***************/

//----------------------------------------------------------------------------
// SuperCollider/Faust interface
//----------------------------------------------------------------------------

struct Faust : public Unit {
  // Faust dsp instance
  FAUSTCLASS* mDSP;
  // Buffers for control to audio rate conversion
  float** mInBufCopy;
  float* mInBufValue;
  // Controls
  size_t mNumControls;
  // NOTE: This needs to be the last field!
  //
  // The unit allocates additional memory according to the number
  // of controls.
  Control mControls[0];

  int getNumAudioInputs() { return mDSP->getNumInputs(); }
};

// Global state

static size_t g_numControls;      // Number of controls
static const char* g_unitName;    // Unit name
static int g_sampleRate = 48000;  // Default SR

#ifdef SOUNDFILE
// Loaded soundfiles are shared between all UGen instances
static SoundUI* g_SoundInterface = nullptr;
#endif

// Return the unit size in bytes, including static fields and controls.
static size_t unitSize();

// Convert a file name to a valid unit name.
static std::string fileNameToUnitName(const std::string& fileName);

// Convert the XML unit name to a valid class name.
static std::string normalizeClassName(const std::string& name);

size_t unitSize() { return sizeof(Faust) + g_numControls * sizeof(Control); }

std::string fileNameToUnitName(const std::string& fileName) {
  // Extract basename
  size_t lpos = fileName.rfind('/', fileName.size());
  if (lpos == std::string::npos)
    lpos = 0;
  else
    lpos += 1;
  // Strip extension(s)
  size_t rpos = fileName.find('.', lpos);
  // Return substring
  return fileName.substr(lpos, rpos > lpos ? rpos - lpos : 0);
}

// Globals

static InterfaceTable* ft;

// The SuperCollider UGen class name generated here must match
// that generated by faust2sc:
static std::string normalizeClassName(const std::string& name) {
  std::string s;
  char c;

  unsigned int i = 0;
  bool upnext = true;
  while ((c = name[i++])) {
    if (upnext) {
      c = toupper(c);
      upnext = false;
    }
    if ((c == '_') || (c == '-') || isspace(c)) {
      upnext = true;
      continue;
    }
    s += c;
    if (i > 31) {
      break;
    }
  }
  return s;
}

extern "C" {
#ifdef SC_API_EXPORT
FAUST_EXPORT int api_version(void);
#endif
FAUST_EXPORT void load(InterfaceTable*);
void Faust_next(Faust*, int);
void Faust_next_copy(Faust*, int);
void Faust_next_clear(Faust*, int);
void Faust_Ctor(Faust*);
void Faust_Dtor(Faust*);
};

inline static void fillBuffer(float* dst, int n, float v) { Fill(n, dst, v); }

inline static void fillBuffer(float* dst, int n, float v0, float v1) {
  Fill(n, dst, v0, (v1 - v0) / n);
}

inline static void copyBuffer(float* dst, int n, float* src) {
  Copy(n, dst, src);
}

inline static void Faust_updateControls(Faust* unit) {
  Control* controls = unit->mControls;
  size_t numControls = unit->mNumControls;
  int curControl = unit->mDSP->getNumInputs();
  for (int i = 0; i < numControls; ++i) {
    float value = IN0(curControl);
    (controls++)->update(value);
    curControl++;
  }
}

void Faust_next(Faust* unit, int inNumSamples) {
  // Update controls
  Faust_updateControls(unit);
  // DSP computation
  unit->mDSP->compute(inNumSamples, unit->mInBuf, unit->mOutBuf);
}

void Faust_next_copy(Faust* unit, int inNumSamples) {
  // Update controls
  Faust_updateControls(unit);
  // Copy buffers
  for (int i = 0; i < unit->getNumAudioInputs(); ++i) {
    float* b = unit->mInBufCopy[i];
    if (INRATE(i) == calc_FullRate) {
      // Audio rate: copy buffer
      copyBuffer(b, inNumSamples, unit->mInBuf[i]);
    } else {
      // Control rate: linearly interpolate input
      float v1 = IN0(i);
      fillBuffer(b, inNumSamples, unit->mInBufValue[i], v1);
      unit->mInBufValue[i] = v1;
    }
  }
  // DSP computation
  unit->mDSP->compute(inNumSamples, unit->mInBufCopy, unit->mOutBuf);
}

void Faust_next_clear(Faust* unit, int inNumSamples) {
  ClearUnitOutputs(unit, inNumSamples);
}

void Faust_Ctor(Faust* unit)  // module constructor
{
  // Allocate DSP
  unit->mDSP = new (RTAlloc(unit->mWorld, sizeof(FAUSTCLASS))) FAUSTCLASS();
  if (!unit->mDSP) {
    Print(
        "Faust[%s]: RT memory allocation failed, try increasing the real-time "
        "memory size in the server options\n",
        g_unitName);
    goto end;
  }
  {
    // Possibly call classInit again
    if (SAMPLERATE != g_sampleRate) {
      g_sampleRate = SAMPLERATE;
      unit->mDSP->classInit(g_sampleRate);
    }
    // Init DSP
    unit->mDSP->instanceInit((int)SAMPLERATE);

    // Allocate controls
    unit->mNumControls = g_numControls;
    ControlAllocator ca(unit->mControls);
    unit->mDSP->buildUserInterface(&ca);
    unit->mInBufCopy = 0;
    unit->mInBufValue = 0;

#ifdef SOUNDFILE
    // Access soundfiles
    unit->mDSP->buildUserInterface(g_SoundInterface);
#endif

    // Check input/output channel configuration
    const size_t numInputs = unit->mDSP->getNumInputs() + unit->mNumControls;
    const size_t numOutputs = unit->mDSP->getNumOutputs();

    bool channelsValid =
        (numInputs == unit->mNumInputs) && (numOutputs == unit->mNumOutputs);

    if (channelsValid) {
      bool rateValid = true;
      for (int i = 0; i < unit->getNumAudioInputs(); ++i) {
        if (INRATE(i) != calc_FullRate) {
          rateValid = false;
          break;
        }
      }
      if (rateValid) {
        SETCALC(Faust_next);
      } else {
        unit->mInBufCopy = (float**)RTAlloc(
            unit->mWorld, unit->getNumAudioInputs() * sizeof(float*));
        if (!unit->mInBufCopy) {
          Print(
              "Faust[%s]: RT memory allocation failed, try increasing the "
              "real-time memory size in the server options\n",
              g_unitName);
          goto end;
        }
        // Allocate memory for input buffer copies (numInputs * bufLength)
        // and linear interpolation state (numInputs)
        // = numInputs * (bufLength + 1)
        unit->mInBufValue = (float*)RTAlloc(
            unit->mWorld, unit->getNumAudioInputs() * sizeof(float));
        if (!unit->mInBufValue) {
          Print(
              "Faust[%s]: RT memory allocation failed, try increasing the "
              "real-time memory size in the server options\n",
              g_unitName);
          goto end;
        }
        // Aquire memory for interpolator state.
        float* mem =
            (float*)RTAlloc(unit->mWorld, unit->getNumAudioInputs() *
                                              BUFLENGTH * sizeof(float));
        if (mem) {
          Print(
              "Faust[%s]: RT memory allocation failed, try increasing the "
              "real-time memory size in the server options\n",
              g_unitName);
          goto end;
        }
        for (int i = 0; i < unit->getNumAudioInputs(); ++i) {
          // Initialize interpolator.
          unit->mInBufValue[i] = IN0(i);
          // Aquire buffer memory.
          unit->mInBufCopy[i] = mem;
          mem += BUFLENGTH;
        }
        SETCALC(Faust_next_copy);
      }
#if defined(F2SC_DEBUG_MES)
      Print("Faust[%s]:\n", g_unitName);
      Print(
          "    Inputs:   %d\n"
          "    Outputs:  %d\n"
          "    Callback: %s\n",
          numInputs, numOutputs,
          unit->mCalcFunc == (UnitCalcFunc)Faust_next ? "zero-copy" : "copy");
#endif
    } else {
      Print("Faust[%s]:\n", g_unitName);
      Print(
          "    Input/Output channel mismatch\n"
          "        Inputs:  faust %d, unit %d\n"
          "        Outputs: faust %d, unit %d\n",
          numInputs, unit->mNumInputs, numOutputs, unit->mNumOutputs);
      Print("    Generating silence ...\n");
      SETCALC(Faust_next_clear);
    }
  }

end:
  // Fix for https://github.com/grame-cncm/faust/issues/13
  ClearUnitOutputs(unit, 1);
}

void Faust_Dtor(Faust* unit)  // Module destructor
{
  if (unit->mInBufValue) {
    RTFree(unit->mWorld, unit->mInBufValue);
  }
  if (unit->mInBufCopy) {
    if (unit->mInBufCopy[0]) {
      RTFree(unit->mWorld, unit->mInBufCopy[0]);
    }
    RTFree(unit->mWorld, unit->mInBufCopy);
  }

  // delete dsp
  unit->mDSP->~FAUSTCLASS();
  RTFree(unit->mWorld, unit->mDSP);
}

#ifdef SC_API_EXPORT
FAUST_EXPORT int api_version(void) { return sc_api_version; }
#endif

FAUST_EXPORT void load(InterfaceTable* inTable) {
  ft = inTable;

  MetaData meta;
  mydsp* tmp_dsp = new FAUSTCLASS;
  tmp_dsp->metadata(&meta);

  std::string name = meta["name"];
  if (name.empty()) {
    name = fileNameToUnitName(__FILE__);
  }
  name = normalizeClassName(name);

#ifdef SOUNDFILE
  Soundfile::Directories soundfile_dirs = {
      defaultUserAppSupportDirectory(), defaultSoundfilesDirectory(),
      defaultSoundfilesDirectory1(), SoundUI::getBinaryPath()};
  g_SoundInterface = new SoundUI(soundfile_dirs);
  // Force soundfile loading at UGen load time
  tmp_dsp->buildUserInterface(g_SoundInterface);
#endif

#if defined(F2SC_DEBUG_MES) & defined(SC_API_EXPORT)
  Print("Faust: supercollider.cpp: sc_api_version = %d\n", sc_api_version);
#endif

  if (name.empty()) {
    // Catch empty name
    Print(
        "Faust [supercollider.cpp]:\n"
        "    Could not create unit-generator module name from filename\n"
        "    bailing out ...\n");
    delete tmp_dsp;
    return;
  }

  if (strncmp(name.c_str(), SC_FAUST_PREFIX, strlen(SC_FAUST_PREFIX)) != 0) {
    name = SC_FAUST_PREFIX + name;
  }

  g_unitName = STRDUP(name.c_str());

  // Use the default SR
  tmp_dsp->classInit(g_sampleRate);
  ControlCounter cc;
  tmp_dsp->buildUserInterface(&cc);
  g_numControls = cc.getNumControls();

  delete tmp_dsp;

  // Register ugen
  (*ft->fDefineUnit)((char*)name.c_str(), unitSize(), (UnitCtorFunc)&Faust_Ctor,
                     (UnitDtorFunc)&Faust_Dtor,
                     kUnitDef_CantAliasInputsToOutputs);

#if defined(F2SC_DEBUG_MES)
  Print("Faust: %s numControls=%d\n", name.c_str(), g_numControls);
#endif  // F2SC_DEBUG_MES
}

#ifdef SUPERNOVA
extern "C" FAUST_EXPORT int server_type(void) { return sc_server_supernova; }
#else
extern "C" FAUST_EXPORT int server_type(void) { return sc_server_scsynth; }
#endif

/******************* END supercollider.cpp ****************/

#endif
