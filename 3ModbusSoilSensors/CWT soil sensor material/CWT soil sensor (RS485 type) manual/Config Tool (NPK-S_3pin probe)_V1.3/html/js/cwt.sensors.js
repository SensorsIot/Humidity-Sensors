//------------------------------------------------------------------
//传感器定义
//只需在此文件定义就可以使用
//------------------------------------------------------------------
var Sensors = [

    //---------------------------------------------------5Pin probe定义开始
    {
        name: "CWT SOIL sensor (NPK-S)",//在下拉中显示的名称
        img: "33.jpg",//显示传感器的图片(imges中)
        slaveAddr: "1",//默认传感器地址
        Registers: [
            {
                regName: "1: Nitrogen (N)",
                regAddr: 31,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
            },
            {
                regName: "2: Phosphorus (P)",
                regAddr: 32,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
            },
            {
                regName: "3: Potassium (K)",
                regAddr: 33,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
            },
            {
                regName: "4: Nitrogen(N) factor",
                regAddr: 1257,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.FloatReverse,
                valueScale: 1,
                write: true,
            },
            {
                regName: "5: Nitrogen(N) offset",
                regAddr: 1259,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
                write: true,
            },
            {
                regName: "6: Phosphorus(P) factor",
                regAddr: 1267,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.FloatReverse,
                valueScale: 1,
                write: true,
            },
            {
                regName: "7: Phosphorus(P) offset",
                regAddr: 1269,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
                write: true,
            },
            {
                regName: "8: Potassium(K) factor",
                regAddr: 1277,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.FloatReverse,
                valueScale: 1,
                write: true,
            },
            {
                regName: "9: Potassium(K) offset",
                regAddr: 1279,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
                write: true,
            },
            {
                regName: "10: Baud",
                regAddr: 2002,
                regType: Modbus.RegType.KEEP,
                valueType: Modbus.ValueType.Signed,
                valueScale: 1,
                write: true,
            },
        ],
        QueryAddrHex: "FF 03 07 D0 00 01 91 59",//查询地址的数据，如果注释掉则无读从机地址的按钮
        slaveRegAddr: 0x7D0,//从机的地址所在的寄存器
        displayBtnWriteAddr: 1,//值0/1,0表示无写从机地址的按钮;1表示有写从机地址的按钮
        Tips: "Red: DC12V  Black: GND  Yellow: A+  Green: B-",
        funBtns: [//功能按钮(写寄存器值)
            {
                //按钮名称
                name: "Reset N P K",
                //params:写寄存器的参数，可定义多个
                params: [
                    // regType:COIL或KEEP
                    // regAddr从1开始
                    // value为10进制值，0-65535(注：0xFFFF=65535)
                    // valueType:Signed,Unsigned,Long,LongReverse,Float,FloatReverse
                    { regType: Modbus.RegType.KEEP, regAddr: 31, value: 65535, valueType: Modbus.ValueType.Unsigned },
                    { regType: Modbus.RegType.KEEP, regAddr: 32, value: 65535, valueType: Modbus.ValueType.Unsigned },
                    { regType: Modbus.RegType.KEEP, regAddr: 33, value: 65535, valueType: Modbus.ValueType.Unsigned },
                    { regType: Modbus.RegType.KEEP, regAddr: 1257, value: 1, valueType: Modbus.ValueType.FloatReverse },
                    { regType: Modbus.RegType.KEEP, regAddr: 1259, value: 0, valueType: Modbus.ValueType.Unsigned },
                    { regType: Modbus.RegType.KEEP, regAddr: 1267, value: 1, valueType: Modbus.ValueType.FloatReverse },
                    { regType: Modbus.RegType.KEEP, regAddr: 1269, value: 0, valueType: Modbus.ValueType.Unsigned },
                    { regType: Modbus.RegType.KEEP, regAddr: 1277, value: 1, valueType: Modbus.ValueType.FloatReverse },
                    { regType: Modbus.RegType.KEEP, regAddr: 1279, value: 0, valueType: Modbus.ValueType.Unsigned },
                ]
            },
            {//按钮2，暂未使用
                name: "",//为空不会显示
                //params:写寄存器的参数，可定义多个
                params: [
                    // regType:COIL或KEEP
                    // regAddr从1开始
                    // value为10进制值，0-65535(注：0xFFFF=65535)
                    // valueType:Signed,Unsigned,Long,LongReverse,Float,FloatReverse
                    // { regType: Modbus.RegType.KEEP, regAddr: 5, value: 65535, valueType: Modbus.ValueType.Unsigned },
                ]
            },
        ]
    },
    //---------------------------------------------------5PIN probe定义结束

];