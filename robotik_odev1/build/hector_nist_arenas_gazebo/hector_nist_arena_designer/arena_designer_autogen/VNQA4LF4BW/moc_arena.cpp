/****************************************************************************
** Meta object code from reading C++ file 'arena.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "model/arena.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'arena.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Arena_t {
    QByteArrayData data[8];
    char stringdata0[79];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Arena_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Arena_t qt_meta_stringdata_Arena = {
    {
QT_MOC_LITERAL(0, 0, 5), // "Arena"
QT_MOC_LITERAL(1, 6, 12), // "elementAdded"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 13), // "ArenaElement*"
QT_MOC_LITERAL(4, 34, 7), // "element"
QT_MOC_LITERAL(5, 42, 14), // "elementRemoved"
QT_MOC_LITERAL(6, 57, 8), // "modified"
QT_MOC_LITERAL(7, 66, 12) // "slotModified"

    },
    "Arena\0elementAdded\0\0ArenaElement*\0"
    "element\0elementRemoved\0modified\0"
    "slotModified"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Arena[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,
       6,    0,   40,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   41,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void Arena::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Arena *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->elementAdded((*reinterpret_cast< ArenaElement*(*)>(_a[1]))); break;
        case 1: _t->elementRemoved((*reinterpret_cast< ArenaElement*(*)>(_a[1]))); break;
        case 2: _t->modified(); break;
        case 3: _t->slotModified(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Arena::*)(ArenaElement * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Arena::elementAdded)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Arena::*)(ArenaElement * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Arena::elementRemoved)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Arena::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Arena::modified)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Arena::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_Arena.data,
    qt_meta_data_Arena,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Arena::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Arena::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Arena.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Arena::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void Arena::elementAdded(ArenaElement * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Arena::elementRemoved(ArenaElement * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Arena::modified()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
