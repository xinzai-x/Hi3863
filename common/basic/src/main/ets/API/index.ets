import { RequestAxios } from "./RequestAPI"

//API需要的数据类型，需与API文档保持一致
// 定义注册数据类型
export interface Register {
  "id": string,
  "username": string,
  "phone": string,
  "token": string,
  "token_expire": Date,
  "host": string,
  "client_id": string,
  "username_mqtt": string,
  "password_mqtt": string,
  "connect_timeout": number,
  "mqtt_version": number,
  "avatar_url": string,
  "gender": string,
  "birth_date": Date,
  "age": number
}

//保存校验手机号和密码
interface loginInfo {
  phone: string,
  password: string
}

// 设备基本信息
export interface deviceBasicInfo {
  id: string;
  user_id: string; // 添加 user_id
  name: string; // 设备名称，比如“卧室灯”或“空调”
  type: string; // 设备类型，比如“light”、“ac”、“sensor”等
  protocol: string; // 通信协议，如 MQTT、BLE、Zigbee 等
  linkstatus: string; //连接状态
  location: string; // 设备所处位置，如“客厅”、“卧室”、“卫生间”等
  created_at: string; // 创建时间
  topics: string; // 订阅主题列表
  qos: number; // 使用 number 替代 MqttQos
}

// MQTT配置信息
export interface mqttInfo {
  id: string; // 服务器ID
  host: string; // 服务器地址
  client_id: string; // 客户端ID
  username: string; // 用户名
  password: string; // 密码
  connect_timeout: number; // 连接超时时间(毫秒)
  mqtt_version: number; // MQTT协议版本
}


// 注册账号，API请求添加数据
export const postRegisterApi = async (data: object) => {
  console.log('参数数据：', JSON.stringify(data))
  const res = await RequestAxios.post<Register>('/users', data)
  if (res.data.code !== 201) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

//登陆校验，只有登录才有返回token
export const getLoginApi = async (phone: string, password: string) => {
  const data: loginInfo = { phone: phone, password: password };
  const res = await RequestAxios.post<Register>('/users/login', data)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 退出登录
export const postLoginApi = async () => {
  const res = await RequestAxios.post<Register>('/users/logout')
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

//根据手机号获取信息
export const getRegisterApi = async (userPhone: string) => {
  const res = await RequestAxios.get<Register>(`/users?phone=${userPhone}`)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 修改用户信息
export const putUserInfo = async (userID: string, data: object) => {
  const res = await RequestAxios.put<Register>(`/users/${userID}`, data)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 删除用户
export const deleteRegisterApi = async (id: string) => {
  const res = await RequestAxios.delete<Register>(`/users/${id}`)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

//添加设备
export const postDeviceApi = async (data: object) => {
  console.log('参数数据：', JSON.stringify(data))
  const res = await RequestAxios.post<deviceBasicInfo>('/devices', data)
  if (res.data.code !== 201) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

//查询全部设备信息
export const getAllDeviceApi = async () => {
  const res = await RequestAxios.get<deviceBasicInfo[]>('/devices')
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 查询单个设备信息
export const getDeviceApi = async (deviceID: string) => {
  const res = await RequestAxios.get<deviceBasicInfo>(`/devices/${deviceID}`)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 搜索设备
export const getSearchDeviceApi = async (name: string) => {
  const res = await RequestAxios.get<deviceBasicInfo[]>(`/devices/search?name=${name}`)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 修改设备信息
export const putDeviceApi = async (deviceID: string, data: object) => {
  const res = await RequestAxios.put<deviceBasicInfo>(`/devices/${deviceID}`, data)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 根据设备id删除设备
export const deleteDeviceApi = async (deviceID: string) => {
  const res = await RequestAxios.delete<deviceBasicInfo>(`/devices/${deviceID}`)
  if (res.data.code !== 200) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

//添加MQTT配置
export const postMQTTApi = async (data: object) => {
  console.log('参数数据：', JSON.stringify(data))
  const res = await RequestAxios.post<mqttInfo>('/mqtt_configs', data)
  if (res.data.code !== 201) {
    console.error('失败：', JSON.stringify(res))
  } else {
    console.log('成功：', JSON.stringify(res))
  }
  return res
}

// 根据用户id获取mqtt配置信息
export const getMQTTApi = async (user_id: string) => {
  const res = await RequestAxios.get<mqttInfo>(`/mqtt_configs/user/${user_id}`)
  if (res.data.code === 200) {
    console.log('成功：', JSON.stringify(res))
  } else {
    console.error('失败：', JSON.stringify(res))
  }
  return res
}

// 根据用户id更新mqtt配置信息
export const putMQTTApi = async (user_id: string, data: object) => {
  const res = await RequestAxios.put<mqttInfo>(`/mqtt_configs/user/${user_id}`, data)
  if (res.data.code === 200) {
    console.log('成功：', JSON.stringify(res))
  } else {
    console.error('失败：', JSON.stringify(res))
  }
  return res
}