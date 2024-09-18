#ifndef MYDLL_GLOBAL_H
#define MYDLL_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(MYDLL_LIBRARY)
#define MYDLL_EXPORT Q_DECL_EXPORT
#else
#define MYDLL_EXPORT Q_DECL_IMPORT
#endif

#endif // MYDLL_GLOBAL_H
