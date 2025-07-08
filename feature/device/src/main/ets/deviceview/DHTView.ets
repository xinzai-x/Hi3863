import { InfoParams, getDeviceApi, MQTTTopics, putDeviceApi, } from "basic";
import { promptAction } from "@kit.ArkUI";
import { MqttQos } from "@ohos/mqtt";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";

// 定义 payload 的类型
interface MqttPayload {
  //发送的格式(因为接收是这个格式)
  temp?: number; // 温度值
  humidity?: number; // 湿度值
}

@Entry
@Component
struct DHTView {
  @State isRefreshing: boolean = false; //下拉刷新开关
  @State temperature: string = '0'; //保存温度数据
  @State humidity: string = '0'; //保存湿度数据
  @State topicsSend: string = ''; //发送主题
  @State messageToSend: string = ''; //发送数据
  //保存设备基础信息
  @State params: InfoParams = {
    deviceId: '', //设备id
    deviceName: '', // 设备名称
    linkMode: '', // 连接方式(MQTT/HTTP)
    icon: '', // 设备图标
  };
  //保存MQTT配置信息
  @State mqtt_info: MQTTTopics = {
    topics: '', // 订阅主题列表
    qos: 0 // 初始化为 0（对应 AT_MOST_ONCE）
  };

  /**
   * 进入页面订阅并调用回调
   */
  async DHT_MQTTInfoDB() {
    // 订阅主题
    try {
      const res = await getDeviceApi(this.params.deviceId)
      if (res.data.code === 200 && res.data.data.topics !== null && res.data.data.qos !== null) {
        const topicsText: string[] = res.data.data.topics.split(',')
          .map(t => t.trim())
          .filter(t => t !== '');
        const subscribeResult =
          await MqttClientWrapper.subscribe(topicsText, res.data.data.qos as MqttQos)
        if (subscribeResult.success) {
          promptAction.openToast({ message: '订阅成功' })
        }
        //接收数据回调
        MqttClientWrapper.setMessageCallback((topic, payload) => {
          console.log(`[DHTView] Received message on topic ${topic}: ${payload}`);
          if (res.data.data.topics === topic) {
            try {
              const data: MqttPayload = JSON.parse(payload);
              console.log('接收数据：', JSON.stringify(data));
              // 更新温度和湿度
              if (data.temp !== undefined) {
                this.temperature = data.temp.toFixed(1); // 保留一位小数
              }
              if (data.humidity !== undefined) {
                this.humidity = data.humidity.toFixed(1);
              }
            } catch (parseError) {
              console.error(`[MQTT] JSON解析失败 (主题: ${topic}):`, parseError);
              console.error('原始payload:', payload);
              promptAction.openToast({ message: '请检查数据格式' })
            }
          }
        });
      } else {
        promptAction.openToast({ message: '订阅失败或没有订阅主题' })
      }
    } catch (error) {
      switch (error.response.status) {
        case 400:
          promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
          break;
        case 401:
          promptAction.openToast({ message: '无有效token' })
          break;
        case 403:
          promptAction.openToast({ message: '无权限修改' })
          break;
        case 500:
          promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
          break;
        default:
          promptAction.openToast({ message: '注册失败，请稍后重试' })
          break;
      }
    }
  }

  /**
   * 构建组件UI
   * 包含：设备状态显示、温湿度数据展示、消息发送、MQTT配置信息展示
   */
  build() {
    NavDestination() {
      Scroll() {
        Column({ space: 10 }) {
          // 设备图标展示
          Image(this.params.icon).height('46%').padding(14)

          // 数据展示区域标题
          Text('数据展示')
            .fontWeight(FontWeight.Bold)
            .margin({ top: 16 })
            .width('100%')
            .fontSize(20)
            .textAlign(TextAlign.Start)

          // 温湿度数据展示区域
          Column({ space: 2 }) {
            // 温湿度数值显示行
            Row() {
              Text(`温度数据：${this.temperature}°C`).fontWeight(FontWeight.Medium).layoutWeight(1)
              Text(`湿度数据：${this.humidity}%`).fontWeight(FontWeight.Medium).layoutWeight(1)
            }.width('100%').justifyContent(FlexAlign.SpaceBetween)

            // 温湿度仪表盘行
            Row() {
              // 温度仪表盘
              Stack() {
                DataPanel({ values: [Number(this.temperature)], max: 100, type: DataPanelType.Circle })
                  .width(136)
                  .height(136)
                  .strokeWidth(16)
                Column() {
                  Text(`${this.temperature}°C`).fontSize(16).fontWeight(FontWeight.Medium)
                  Text('温度').fontSize(9.33).lineHeight(12.83).fontWeight(500).opacity(0.6)
                }
              }

              // 湿度仪表盘
              Stack() {
                DataPanel({ values: [Number(this.humidity)], max: 100, type: DataPanelType.Circle })
                  .width(136)
                  .height(136)
                  .strokeWidth(16)
                Column() {
                  Text(`${this.humidity}%`).fontSize(16).fontWeight(FontWeight.Medium)
                  Text('湿度').fontSize(9.33).lineHeight(12.83).fontWeight(500).opacity(0.6)
                }
              }
            }.width('100%').justifyContent(FlexAlign.SpaceBetween)
          }
          .width('100%')
          .backgroundColor($r('app.color.white'))
          .height(160)
          .borderRadius(16)
          .padding(16)

          Text('订阅主题')
            .fontWeight(FontWeight.Bold)
            .margin({ top: 16 })
            .width('100%')
            .fontSize(20)
            .textAlign(TextAlign.Start)

          Column({ space: 12 }) {
            Row() {
              Text('订阅主题：').fontWeight(FontWeight.Medium)
              TextArea({ text: $$this.mqtt_info.topics }).layoutWeight(1)
            }.width('100%')

            Row() {
              Text('QoS等级：').fontWeight(FontWeight.Medium)
              TextArea({ text: $$this.mqtt_info.qos }).layoutWeight(1)
            }.width('100%')

            Button('订阅').width('100%').fontWeight(FontWeight.Medium)
              .onClick(async () => {
                //将字符串按,拆解成字符数组存入
                // this.mqtt_info.topics = this.topicsText.split(',')
                //   .map(t => t.trim())
                //   .filter(t => t !== '');
                this.getUIContext().getFocusController().clearFocus(); //收起软键盘
                if (this.mqtt_info.topics.length > 0) {
                  console.log(`温湿度订阅主题: ${this.mqtt_info.topics}`);
                  try {
                    const res = await putDeviceApi(this.params.deviceId, this.mqtt_info)
                    if (res.data.code === 200) {
                      const topicsText: string[] = this.mqtt_info.topics.split(',')
                        .map(t => t.trim())
                        .filter(t => t !== '');
                      const subscribeResult =
                        await MqttClientWrapper.subscribe(topicsText, this.mqtt_info.qos as MqttQos)
                      if (subscribeResult.success) {
                        promptAction.openToast({ message: '订阅成功' })
                      } else {
                        promptAction.openToast({ message: '订阅失败' })
                      }
                    }
                  } catch (error) {
                    switch (error.response.status) {
                      case 400:
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 403:
                        promptAction.openToast({ message: '无权限修改' })
                        break;
                      case 500:
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      default:
                        promptAction.openToast({ message: '订阅失败，请稍后重试' })
                        break;
                    }
                  }
                } else {
                  promptAction.openToast({ message: '无订阅主题，请配置主题' });
                }
              })
          }
          .width('100%')
          .backgroundColor($r('app.color.white'))
          .height(170)
          .borderRadius(16)
          .padding(16)

          // 消息发送区域标题
          Text('消息发送')
            .fontWeight(FontWeight.Bold)
            .margin({ top: 16 })
            .width('100%')
            .fontSize(20)
            .textAlign(TextAlign.Start)

          // 消息发送区域
          Column({ space: 14 }) {
            Row() {
              Text('发送主题：').fontWeight(FontWeight.Medium)
              TextArea({ text: $$this.topicsSend }).layoutWeight(1)
            }.width('100%')

            Row() {
              Text('消息发送：').fontWeight(FontWeight.Medium)
              TextArea({ placeholder: 'eg:{"led": "on"}', text: $$this.messageToSend }).layoutWeight(1)
            }.width('100%')

            // 发送按钮
            Button('发送').width('100%').fontWeight(FontWeight.Medium)
              .onClick(async () => {
                this.getUIContext().getFocusController().clearFocus(); //收起软键盘
                if (!this.topicsSend) {
                  promptAction.openToast({ message: '请输入发送主题' });
                  return;
                }
                if (!this.messageToSend) {
                  promptAction.openToast({ message: '请输入消息内容' });
                  return;
                }
                // 发送MQTT消息
                const topic = this.topicsSend.trim();
                const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                const result = await MqttClientWrapper.publish(topic, this.messageToSend, qos);
                promptAction.openToast({ message: result.success ? '消息发送成功' : `发送失败: ${result.error}` });
              })
          }
          .width('100%')
          .backgroundColor($r('app.color.white'))
          .height(180)
          .borderRadius(16)
          .padding(16)
        }.width('100%').padding(16)
      }
      .width('100%')
      .height('100%')
      .scrollBar(BarState.Off)
      .edgeEffect(EdgeEffect.Spring)
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
    }.width('100%').title(this.params.deviceName)
    .backgroundColor($r('app.color.common_gray_bg'))
    .onReady(async (context: NavDestinationContext) => {
      //接收页面传递进来的数据
      const pathStack = context.pathStack;
      const paramsArray = pathStack.getParamByName('DHTInfoView') as InfoParams[];
      if (paramsArray && paramsArray.length > 0) {
        const param = paramsArray[paramsArray.length - 1];
        this.params.deviceId = param.deviceId;
        this.params.deviceName = param.deviceName;
        this.params.icon = param.icon;
        this.params.linkMode = param.linkMode;
        console.log(`[DHTView] Received params: ${JSON.stringify(this.params)}`);
        await this.DHT_MQTTInfoDB();
      }
    })
  }
}

export default DHTView