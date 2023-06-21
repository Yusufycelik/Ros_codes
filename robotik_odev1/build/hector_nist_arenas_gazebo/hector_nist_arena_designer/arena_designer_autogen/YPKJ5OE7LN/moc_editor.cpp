/****************************************************************************
** Meta object code from reading C++ file 'editor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ui/editor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'editor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Editor_t {
    QByteArrayData data[19];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Editor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Editor_t qt_meta_stringdata_Editor = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Editor"
QT_MOC_LITERAL(1, 7, 19), // "slotRotateClockwise"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 26), // "slotRotateCounterClockwise"
QT_MOC_LITERAL(4, 55, 10), // "slotRemove"
QT_MOC_LITERAL(5, 66, 25), // "slotItemMountPointChanged"
QT_MOC_LITERAL(6, 92, 5), // "index"
QT_MOC_LITERAL(7, 98, 7), // "slotNew"
QT_MOC_LITERAL(8, 106, 8), // "slotOpen"
QT_MOC_LITERAL(9, 115, 8), // "slotSave"
QT_MOC_LITERAL(10, 124, 10), // "slotSaveAs"
QT_MOC_LITERAL(11, 135, 10), // "slotExport"
QT_MOC_LITERAL(12, 146, 13), // "slotExportSdf"
QT_MOC_LITERAL(13, 160, 20), // "slotSelectionChanged"
QT_MOC_LITERAL(14, 181, 21), // "slotShowDocumentation"
QT_MOC_LITERAL(15, 203, 22), // "slotElementTypeHovered"
QT_MOC_LITERAL(16, 226, 13), // "ArenaElement*"
QT_MOC_LITERAL(17, 240, 7), // "element"
QT_MOC_LITERAL(18, 248, 12) // "slotModified"

    },
    "Editor\0slotRotateClockwise\0\0"
    "slotRotateCounterClockwise\0slotRemove\0"
    "slotItemMountPointChanged\0index\0slotNew\0"
    "slotOpen\0slotSave\0slotSaveAs\0slotExport\0"
    "slotExportSdf\0slotSelectionChanged\0"
    "slotShowDocumentation\0slotElementTypeHovered\0"
    "ArenaElement*\0element\0slotModified"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Editor[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   84,    2, 0x0a /* Public */,
       3,    0,   85,    2, 0x0a /* Public */,
       4,    0,   86,    2, 0x0a /* Public */,
       5,    1,   87,    2, 0x0a /* Public */,
       7,    0,   90,    2, 0x0a /* Public */,
       8,    0,   91,    2, 0x0a /* Public */,
       9,    0,   92,    2, 0x0a /* Public */,
      10,    0,   93,    2, 0x0a /* Public */,
      11,    0,   94,    2, 0x0a /* Public */,
      12,    0,   95,    2, 0x0a /* Public */,
      13,    0,   96,    2, 0x0a /* Public */,
      14,    0,   97,    2, 0x0a /* Public */,
      15,    1,   98,    2, 0x0a /* Public */,
      18,    0,  101,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,

       0        // eod
};

void Editor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Editor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->slotRotateClockwise(); break;
        case 1: _t->slotRotateCounterClockwise(); break;
        case 2: _t->slotRemove(); break;
        case 3: _t->slotItemMountPointChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->slotNew(); break;
        case 5: _t->slotOpen(); break;
        case 6: _t->slotSave(); break;
        case 7: _t->slotSaveAs(); break;
        case 8: _t->slotExport(); break;
        case 9: _t->slotExportSdf(); break;
        case 10: _t->slotSelectionChanged(); break;
        case 11: _t->slotShowDocumentation(); break;
        case 12: _t->slotElementTypeHovered((*reinterpret_cast< ArenaElement*(*)>(_a[1]))); break;
        case 13: _t->slotModified(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Editor::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_Editor.data,
    qt_meta_data_Editor,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Editor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Editor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Editor.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int Editor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
