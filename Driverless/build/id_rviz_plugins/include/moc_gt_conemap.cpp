/****************************************************************************
** Meta object code from reading C++ file 'gt_conemap.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/id_rviz_plugins/include/gt_conemap.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gt_conemap.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay_t {
    QByteArrayData data[3];
    char stringdata0[49];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay_t qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay = {
    {
QT_MOC_LITERAL(0, 0, 32), // "id::rviz_plugins::ConeMapDisplay"
QT_MOC_LITERAL(1, 33, 14), // "updateProperty"
QT_MOC_LITERAL(2, 48, 0) // ""

    },
    "id::rviz_plugins::ConeMapDisplay\0"
    "updateProperty\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_id__rviz_plugins__ConeMapDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void id::rviz_plugins::ConeMapDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ConeMapDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateProperty(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject id::rviz_plugins::ConeMapDisplay::staticMetaObject = { {
    &rviz_common::RosTopicDisplay<imperial_driverless_interfaces::msg::ConeMap>::staticMetaObject,
    qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay.data,
    qt_meta_data_id__rviz_plugins__ConeMapDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *id::rviz_plugins::ConeMapDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *id::rviz_plugins::ConeMapDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_id__rviz_plugins__ConeMapDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::RosTopicDisplay<imperial_driverless_interfaces::msg::ConeMap>::qt_metacast(_clname);
}

int id::rviz_plugins::ConeMapDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::RosTopicDisplay<imperial_driverless_interfaces::msg::ConeMap>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
