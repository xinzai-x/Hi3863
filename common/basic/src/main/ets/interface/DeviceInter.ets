//接收API数据接口
export interface deviceInfoInterface {
  deviceViewId: string //ID
  deviceName: string // 设备名称，例如 "空调"、"客厅灯"
  deviceType: string // 设备类型标识，例如 "ac"、"light"，用于渲染 icon
  pipeline: string // 通信协议，如 "MQTT"、"Zigbee"、"BLE"
  location: string // 安装位置，如 "卧室"、"客厅"
  linkstatus: string // 当前设备连接状态，如 "已连接"、"未连接"
}

//传入设备子页面信息接口
export interface InfoParams {
  deviceId: string;
  deviceName: string;
  linkMode: string;
  icon: string;
}

// MQTT 订阅主题
export interface MQTTTopics {
  topics: string; // 订阅主题列表
  qos: number; // 使用 number 替代 MqttQos
}

export interface DataReloadListener {
  onDataReloaded: () => void
}
