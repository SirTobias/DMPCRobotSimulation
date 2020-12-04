#ifndef SIMSHAREDLIB_H
#define SIMSHAREDLIB_H

#include <QtCore/QtGlobal>

#if defined (SIM_CORE_SHARED_LIB)
#  define SIM_CORE_EXPORT Q_DECL_EXPORT
#else
#  define SIM_CORE_EXPORT Q_DECL_IMPORT
#endif

#endif // SIMSHAREDLIB_H
