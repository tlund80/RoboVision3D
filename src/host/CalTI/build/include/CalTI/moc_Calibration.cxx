/****************************************************************************
** Meta object code from reading C++ file 'Calibration.hpp'
**
** Created: Fri May 23 19:14:29 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/CalTI/Calibration.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Calibration.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dti__Calibration[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      18,   17,   17,   17, 0x05,
      41,   36,   17,   17, 0x05,
      71,   67,   17,   17, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_dti__Calibration[] = {
    "dti::Calibration\0\0updateGuiSignal()\0"
    "imgQ\0annotateGuiSignal(QImage)\0msg\0"
    "consoleSignal(QString)\0"
};

void dti::Calibration::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Calibration *_t = static_cast<Calibration *>(_o);
        switch (_id) {
        case 0: _t->updateGuiSignal(); break;
        case 1: _t->annotateGuiSignal((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 2: _t->consoleSignal((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData dti::Calibration::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dti::Calibration::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_dti__Calibration,
      qt_meta_data_dti__Calibration, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dti::Calibration::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dti::Calibration::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dti::Calibration::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dti__Calibration))
        return static_cast<void*>(const_cast< Calibration*>(this));
    return QObject::qt_metacast(_clname);
}

int dti::Calibration::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void dti::Calibration::updateGuiSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void dti::Calibration::annotateGuiSignal(QImage _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dti::Calibration::consoleSignal(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
