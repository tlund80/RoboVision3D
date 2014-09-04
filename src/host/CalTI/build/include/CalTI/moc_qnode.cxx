/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created: Sat May 24 12:43:48 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/CalTI/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CalTI__QNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   14,   13,   13, 0x05,
      40,   14,   13,   13, 0x05,
      65,   13,   13,   13, 0x05,
      89,   13,   13,   13, 0x05,
     118,  114,   13,   13, 0x05,
     141,   13,   13,   13, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_CalTI__QNode[] = {
    "CalTI::QNode\0\0,\0updateImage(QImage,int)\0"
    "updateTCP(tiv::pose,int)\0"
    "updateImageLeft(QImage)\0"
    "updateImageRight(QImage)\0msg\0"
    "consoleSignal(QString)\0rosShutdown()\0"
};

void CalTI::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->updateImage((*reinterpret_cast< const QImage(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->updateTCP((*reinterpret_cast< const tiv::pose(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->updateImageLeft((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 3: _t->updateImageRight((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 4: _t->consoleSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->rosShutdown(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CalTI::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CalTI::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_CalTI__QNode,
      qt_meta_data_CalTI__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CalTI::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CalTI::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CalTI::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CalTI__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int CalTI::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void CalTI::QNode::updateImage(const QImage & _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CalTI::QNode::updateTCP(const tiv::pose & _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CalTI::QNode::updateImageLeft(const QImage & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CalTI::QNode::updateImageRight(const QImage & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void CalTI::QNode::consoleSignal(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void CalTI::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}
QT_END_MOC_NAMESPACE
