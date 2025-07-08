import { MqttAsync, MqttClient, MqttQos, MqttPersistenceType } from '@ohos/mqtt';

// MQTT操作结果接口
interface MqttResult {
  success: boolean;
  error?: string;
}

// MQTT连接参数接口
interface ConnectParams {
  userName: string;
  password: string;
  connectTimeout: number;
  MQTTVersion: number;
}

// MQTT客户端配置接口
interface ClientConfig {
  serverUrl: string;
  clientId: string;
  persistenceType?: MqttPersistenceType;
}

// 订阅信息结构体
interface Subscription {
  topic: string;
  qos: MqttQos;
}

// 静态MQTT客户端包装类
export default class MqttClientWrapper {
  private static mqttClient: MqttClient | undefined;
  private static subscriptions: Map<string, Subscription> = new Map();
  private static messageCallback: ((topic: string, payload: string) => void) | undefined;
  private static isInitialized: boolean = false;
  private static hasMessageCallback: boolean = false;

  // 防止实例化
  private constructor() {
    throw new Error('MqttClientWrapper is a static class and cannot be instantiated.');
  }

  /**
   * 初始化MQTT客户端
   * @param config 配置项
   */
  static initialize(config: ClientConfig): MqttResult {
    if (MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端已初始化')
      return { success: false, error: 'MQTT客户端已初始化' };
    }

    try {
      MqttClientWrapper.mqttClient = MqttAsync.createMqtt({
        url: config.serverUrl,
        clientId: config.clientId,
        persistenceType: config.persistenceType ?? 1
      });
      MqttClientWrapper.isInitialized = true;
      console.info('[MQTT] 客户端初始化成功');
      return { success: true };
    } catch (error) {
      console.error('[MQTT] 初始化失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 连接MQTT服务
   * @param params 参数
   */
  static async connect(params: ConnectParams): Promise<MqttResult> {
    if (!MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端未初始化，连接失败')
      return { success: false, error: '客户端未初始化，请先调用 initialize' };
    }
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端未创建，连接失败')
      return { success: false, error: '客户端未创建' };
    }

    try {
      console.info('[MQTT] 尝试连接:', params.userName);
      const result = await MqttClientWrapper.mqttClient.connect(params);
      if (result) {
        console.info('[MQTT] 连接成功');
        return { success: true };
      }
      return { success: false, error: '连接返回false' };
    } catch (error) {
      console.error('[MQTT] 连接失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 订阅主题
   * @param topics 单个或多个主题
   * @param qos QoS等级（默认0）
   */
  static async subscribe(topics: string | string[], qos: MqttQos = 0): Promise<MqttResult> {
    if (!MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端未初始化，订阅失败')
      return { success: false, error: '客户端未初始化，请先调用 initialize' };
    }
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端未创建，订阅失败')
      return { success: false, error: '客户端未创建' };
    }

    const topicList = Array.isArray(topics) ? topics : [topics];

    try {
      if (!MqttClientWrapper.hasMessageCallback) {
        MqttClientWrapper.mqttClient.messageArrived((err, data) => {
          if (err) {
            console.error('[MQTT] 接收失败:', MqttClientWrapper.getErrorMessage(err));
          } else if (data && MqttClientWrapper.messageCallback) {
            MqttClientWrapper.messageCallback(data.topic ?? '', data.payload ?? '');
          }
        });
        MqttClientWrapper.hasMessageCallback = true;
        console.info('[MQTT] 消息到达回调已设置');
      }

      for (const topic of topicList) {
        await MqttClientWrapper.mqttClient.subscribe({ topic, qos });
        MqttClientWrapper.subscriptions.set(topic, { topic, qos });
        console.info(`[MQTT] 订阅成功: ${topic} (QoS: ${qos})`);
      }

      return { success: true };
    } catch (error) {
      console.error('[MQTT] 订阅失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 取消订阅
   * @param topics 单个或多个主题
   */
  static async unsubscribe(topics: string | string[]): Promise<MqttResult> {
    if (!MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端未初始化，取消订阅失败')
      return { success: false, error: '客户端未初始化，请先调用 initialize' };
    }
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端未创建，取消订阅失败')
      return { success: false, error: '客户端未创建' };
    }

    const topicList = Array.isArray(topics) ? topics : [topics];

    try {
      for (const topic of topicList) {
        const sub = MqttClientWrapper.subscriptions.get(topic);
        if (sub) {
          await MqttClientWrapper.mqttClient.unsubscribe({ topic, qos: sub.qos });
          MqttClientWrapper.subscriptions.delete(topic);
          console.info(`[MQTT] 取消订阅: ${topic}`);
        }
      }
      return { success: true };
    } catch (error) {
      console.error('[MQTT] 取消订阅失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 发布消息
   * @param topic 主题
   * @param payload 消息内容
   * @param qos QoS等级（默认0）
   * @param retained 是否保留消息
   */
  static async publish(topic: string, payload: string, qos: MqttQos = 0, retained = false): Promise<MqttResult> {
    if (!MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端未初始化，发送消息失败')
      return { success: false, error: '客户端未初始化，请先调用 initialize' };
    }
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端未创建，发送消息失败')
      return { success: false, error: '客户端未创建' };
    }

    try {
      await new Promise<void>((resolve, reject) => {
        MqttClientWrapper.mqttClient!.publish(
          {
            topic,
            payload,
            qos,
            retained
          },
          (err) => {
            if (err) {
              reject(err);
            } else {
              resolve();
            }
          }
        );
      });
      console.info(`[MQTT] 发布成功: ${topic}`);
      return { success: true };
    } catch (error) {
      console.error('[MQTT] 发布失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 断开连接
   */
  static async disconnect(): Promise<MqttResult> {
    if (!MqttClientWrapper.isInitialized) {
      console.error('MQTT客户端未初始化，断开连接失败')
      return { success: false, error: '客户端未初始化，请先调用 initialize' };
    }
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端未创建，断开连接失败')
      return { success: false, error: '客户端未创建' };
    }

    try {
      await MqttClientWrapper.mqttClient.disconnect();
      console.info('[MQTT] 断开连接成功');
      return { success: true };
    } catch (error) {
      console.error('[MQTT] 断开连接失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 销毁客户端
   */
  static async destroy(): Promise<MqttResult> {
    if (!MqttClientWrapper.mqttClient) {
      console.error('MQTT客户端不存在，销毁客户端失败')
      return { success: false, error: '客户端不存在' };
    }

    try {
      await MqttClientWrapper.mqttClient.destroy();
      MqttClientWrapper.mqttClient = undefined;
      MqttClientWrapper.subscriptions.clear();
      MqttClientWrapper.messageCallback = undefined;
      MqttClientWrapper.hasMessageCallback = false;
      MqttClientWrapper.isInitialized = false;
      console.info('[MQTT] 客户端已销毁');
      return { success: true };
    } catch (error) {
      console.error('[MQTT] 销毁失败:', MqttClientWrapper.getErrorMessage(error));
      return { success: false, error: MqttClientWrapper.getErrorMessage(error) };
    }
  }

  /**
   * 设置消息回调
   * @param callback 回调函数
   */
  static setMessageCallback(callback: (topic: string, payload: string) => void) {
    MqttClientWrapper.messageCallback = callback;
    console.info('[MQTT] 消息回调已设置');
  }

  /**
   * 获取所有订阅信息
   */
  static getAllSubscriptions(): Subscription[] {
    return Array.from(MqttClientWrapper.subscriptions.values());
  }

  /**
   * 提取错误信息
   */
  private static getErrorMessage(error: Error | string | object | undefined | null): string {
    if (error instanceof Error) {
      return error.message;
    }
    if (typeof error === 'string') {
      return error;
    }
    try {
      return JSON.stringify(error);
    } catch {
      return '未知错误';
    }
  }
}