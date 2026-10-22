// 跨线程信号用到的值类型必须注册给 Qt 元系统。
//
// 单独放一个头文件，是因为 Qt 的 moc 会在每个 Q_OBJECT 里生成
// QMetaType::fromType<T>() 的表；如果 Q_DECLARE_METATYPE 只写在某一个头里，
// 多个 moc 文件拼进 mocs_compilation.cpp 时会先实例化、后特化，编译直接报
// "specialization after instantiation"。让所有用到这些类型的 Q_OBJECT 头
// 都先包含本文件即可。
#pragma once

#include <QMetaType>

#include "fleet_gui/monitor_core.hpp"

Q_DECLARE_METATYPE(fleet::monitor::TaskView)
Q_DECLARE_METATYPE(fleet::monitor::RobotView)
Q_DECLARE_METATYPE(fleet::monitor::StatusEvent)
Q_DECLARE_METATYPE(fleet::monitor::MapData)