/****************************************************************************
** Meta object code from reading C++ file 'CalibThread.hpp'
**
** Created: Fri May 23 19:14:29 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/CalTI/CalibThread.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CalibThread.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dti__CalibThread[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: signature, parameters, type, tag, flags
      22,   18,   17,   17, 0x05,
      45,   17,   17,   17, 0x05,
      58,   17,   17,   17, 0x05,
      71,   17,   17,   17, 0x05,
      99,   88,   17,   17, 0x05,
     133,   17,   17,   17, 0x05,
     158,   17,   17,   17, 0x05,

 // slots: signature, parameters, type, tag, flags
     228,  189,   17,   17, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_dti__CalibThread[] = {
    "dti::CalibThread\0\0msg\0consoleSignal(QString)\0"
    "imgLSignal()\0imgRSignal()\0finishedSignal()\0"
    "imgQ,title\0annotateGuiSignal(QImage,QString)\0"
    "updateGuiPreviewSignal()\0"
    "updateGuiIntrinsicValsSignal()\0"
    "CAM,intrinsicMatName,distortionMatName\0"
    "readIntrinsicGuessPath(dti::CameraID,QString,QString)\0"
};

void dti::CalibThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CalibThread *_t = static_cast<CalibThread *>(_o);
        switch (_id) {
        case 0: _t->consoleSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->imgLSignal(); break;
        case 2: _t->imgRSignal(); break;
        case 3: _t->finishedSignal(); break;
        case 4: _t->annotateGuiSignal((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 5: _t->updateGuiPreviewSignal(); break;
        case 6: _t->updateGuiIntrinsicValsSignal(); break;
        case 7: _t->readIntrinsicGuessPath((*reinterpret_cast< dti::CameraID(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData dti::CalibThread::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dti::CalibThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_dti__CalibThread,
      qt_meta_data_dti__CalibThread, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dti::CalibThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dti::CalibThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dti::CalibThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dti__CalibThread))
        return static_cast<void*>(const_cast< CalibThread*>(this));
    return QThread::qt_metacast(_clname);
}

int dti::CalibThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void dti::CalibThread::consoleSignal(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dti::CalibThread::imgLSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void dti::CalibThread::imgRSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void dti::CalibThread::finishedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void dti::CalibThread::annotateGuiSignal(QImage _t1, QString _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void dti::CalibThread::updateGuiPreviewSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}

// SIGNAL 6
void dti::CalibThread::updateGuiIntrinsicValsSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}
QT_END_MOC_NAMESPACE
