import { relationalStore } from "@kit.ArkData";

//MQTT设备数据库配置项
export const DB_Device_Configuration: relationalStore.StoreConfig = {
  name: 'dbDeviceConfig.db', //数据名字
  securityLevel: relationalStore.SecurityLevel.S1,
  encrypt: false,
  customDir: 'customDir/subCustomDir',
  isReadOnly: false
}

//表名 DEVICE_Basics_CONFIG
export const Device_CREATE_TABLE = `
CREATE TABLE IF NOT EXISTS DEVICE_Basics_CONFIG (
  ID TEXT PRIMARY KEY NOT NULL,                    -- 设备唯一ID（主键）
  DEVICE_NAME TEXT NOT NULL,                       -- 设备名字
  DEVICE_TYPE TEXT NOT NULL,                       -- 设备类型
  PROTOCOL TEXT NOT NULL,                          -- 连接协议（如 BLE / Wi-Fi）
  LINKSTATUS TEXT NOT NULL,                        -- 连接状态（默认未连接）
  LOCATION TEXT                                    -- 设备位置
);
`;

//表名 MQTT_DEVICE_CONFIG
export const MQTT_CREATE_TABLE = `
CREATE TABLE IF NOT EXISTS MQTT_DEVICE_CONFIG (
  SERVER_ID TEXT PRIMARY KEY NOT NULL,             -- 设备唯一ID（主键）
  SERVER_URI TEXT NOT NULL,                        -- MQTT服务器地址
  CLIENT_ID TEXT NOT NULL,                         -- 客户端ID
  USERNAME TEXT,                                   -- 用户名
  PASSWORD TEXT,                                   -- 密码
  TIMEOUT INTEGER DEFAULT 30,                      -- 超时时间（单位：秒）
  MQTT_VERSION INTEGER DEFAULT 4,                  -- MQTT版本（3=3.1, 4=3.1.1, 5=5.0）
  SUB_TOPICS TEXT,                        -- 订阅主题数组，JSON字符串，如 ["topic1", "topic2"]
  QOS_LEVELS TEXT                       -- QoS等级数组，JSON字符串，如 [0, 1]
);
`;


