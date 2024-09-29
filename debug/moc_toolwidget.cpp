/****************************************************************************
** Meta object code from reading C++ file 'toolwidget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../component/toolwidget.h"
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'toolwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSToolWidgetENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSToolWidgetENDCLASS = QtMocHelpers::stringData(
    "ToolWidget",
    "onConstructPoint",
    "",
    "onConstructLine",
    "onConstructCircle",
    "onConstructPlane",
    "onConstructRectangle",
    "onConstructCylinder",
    "onConstructCone",
    "onConstructSphere"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSToolWidgetENDCLASS_t {
    uint offsetsAndSizes[20];
    char stringdata0[11];
    char stringdata1[17];
    char stringdata2[1];
    char stringdata3[16];
    char stringdata4[18];
    char stringdata5[17];
    char stringdata6[21];
    char stringdata7[20];
    char stringdata8[16];
    char stringdata9[18];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSToolWidgetENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSToolWidgetENDCLASS_t qt_meta_stringdata_CLASSToolWidgetENDCLASS = {
    {
        QT_MOC_LITERAL(0, 10),  // "ToolWidget"
        QT_MOC_LITERAL(11, 16),  // "onConstructPoint"
        QT_MOC_LITERAL(28, 0),  // ""
        QT_MOC_LITERAL(29, 15),  // "onConstructLine"
        QT_MOC_LITERAL(45, 17),  // "onConstructCircle"
        QT_MOC_LITERAL(63, 16),  // "onConstructPlane"
        QT_MOC_LITERAL(80, 20),  // "onConstructRectangle"
        QT_MOC_LITERAL(101, 19),  // "onConstructCylinder"
        QT_MOC_LITERAL(121, 15),  // "onConstructCone"
        QT_MOC_LITERAL(137, 17)   // "onConstructSphere"
    },
    "ToolWidget",
    "onConstructPoint",
    "",
    "onConstructLine",
    "onConstructCircle",
    "onConstructPlane",
    "onConstructRectangle",
    "onConstructCylinder",
    "onConstructCone",
    "onConstructSphere"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSToolWidgetENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   62,    2, 0x0a,    1 /* Public */,
       3,    0,   63,    2, 0x0a,    2 /* Public */,
       4,    0,   64,    2, 0x0a,    3 /* Public */,
       5,    0,   65,    2, 0x0a,    4 /* Public */,
       6,    0,   66,    2, 0x0a,    5 /* Public */,
       7,    0,   67,    2, 0x0a,    6 /* Public */,
       8,    0,   68,    2, 0x0a,    7 /* Public */,
       9,    0,   69,    2, 0x0a,    8 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject ToolWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CLASSToolWidgetENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSToolWidgetENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSToolWidgetENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ToolWidget, std::true_type>,
        // method 'onConstructPoint'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructLine'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructCircle'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructPlane'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructRectangle'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructCylinder'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructCone'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConstructSphere'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void ToolWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ToolWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onConstructPoint(); break;
        case 1: _t->onConstructLine(); break;
        case 2: _t->onConstructCircle(); break;
        case 3: _t->onConstructPlane(); break;
        case 4: _t->onConstructRectangle(); break;
        case 5: _t->onConstructCylinder(); break;
        case 6: _t->onConstructCone(); break;
        case 7: _t->onConstructSphere(); break;
        default: ;
        }
    }
    (void)_a;
}

const QMetaObject *ToolWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ToolWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSToolWidgetENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ToolWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
