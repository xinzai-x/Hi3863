import { deviceBasicInfo, deviceInfoInterface, getAllDeviceApi, PreferenceManagerFactory } from "..";

// 数据变更监听器接口，允许外部注册监听器来响应数据变化
interface DataChangeListener {
  // 数据整体重新加载时调用
  onDataReloaded?: () => void;

  // 数据新增时调用，传入新增项的索引
  onDataAdd?: (index: number) => void;

  // 数据删除时调用，传入删除项的索引
  onDataDelete?: (index: number) => void;

  // 某一项数据发生变化时调用，传入其索引
  onDataChange?: (index: number) => void;
}

// 通用数据源封装类，用于实现 LazyForEach 所需的 IDataSource 接口兼容结构
export class SimpleDataSource<T> {
  private data: T[]; // 内部保存的数据列表
  private listeners: DataChangeListener[] = []; // 数据变更监听器列表

  constructor(data: T[]) {
    this.data = data;
  }

  // 获取当前数据总数量
  totalCount(): number {
    return this.data.length;
  }

  // 根据索引获取数据项
  getData(index: number): T {
    return this.data[index];
  }

  // 注册一个监听器，用于响应数据更新
  registerDataChangeListener(listener: DataChangeListener): void {
    this.listeners.push(listener);
  }

  // 注销一个监听器
  unregisterDataChangeListener(listener: DataChangeListener): void {
    this.listeners = this.listeners.filter(fn => fn !== listener);
  }

  // 更新内部数据并通知监听器数据已重新加载
  updateData(newData: T[]): void {
    this.data = newData;
    this.listeners.forEach(listener => {
      listener.onDataReloaded?.(); // 安全调用监听器回调函数
    });
  }
}

// 设备小组件循环数据更新
export class DeviceStateManager {
  // 静态数据源，用于存储设备信息
  static dataSource: SimpleDataSource<deviceInfoInterface> = new SimpleDataSource<deviceInfoInterface>([]);

  /**
   * 从API加载设备列表并更新数据源
   * @async
   */
  static async loadDeviceList() {
    const res = await getAllDeviceApi();
    console.log('API返回设备信息：', JSON.stringify(res.data));
    const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')

    if (res.data.code === 200 && userPref.getValue('token') !== '') {
      // res.data.data 是 deviceBasicInfo[] 类型
      const deviceList: deviceBasicInfo[] = res.data.data
      const mapped = deviceList.map((item: deviceBasicInfo): deviceInfoInterface => ({
        deviceViewId: item.id,
        deviceName: item.name ?? '',
        deviceType: item.type ?? '',
        pipeline: item.protocol ?? '',
        linkstatus: item.linkstatus ?? '',
        location: item.location ?? ''
      }));

      console.log('回传后的数据：', JSON.stringify(mapped));
      // 使用映射后的数据更新数据源
      DeviceStateManager.dataSource.updateData(mapped);
    } else {
      console.error('API请求失败:', JSON.stringify(res.data));
      DeviceStateManager.dataSource.updateData([]); // 失败时清空数据源
    }
  }

  /**
   * 获取当前设备数据源
   * @returns {SimpleDataSource<deviceInfoInterface>} 设备数据源实例
   */
  static getDataSource(): SimpleDataSource<deviceInfoInterface> {
    return DeviceStateManager.dataSource;
  }
}

export function getIconByType(type: string): string {
  switch (type) {
    case 'LED灯':
      return 'https://i.postimg.cc/DwmPSdjz/led.png';
    case '温湿度传感器':
      return 'https://i.postimg.cc/PJtmTmvT/humiture.png';
    case '蜂鸣器':
      return 'https://i.postimg.cc/xjg9rVJs/buzzer.png';
    case '光照传感器':
      return 'https://i.postimg.cc/MpKN12Lj/BH.png';
    case '直流电机':
      return 'https://i.postimg.cc/BbbPxv2V/ele-mach.png';
    default:
      return 'https://i.postimg.cc/kG1tCLdv/unknown-device.png'; // 提供一个默认图标资源
  }
}