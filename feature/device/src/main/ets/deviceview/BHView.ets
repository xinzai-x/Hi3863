import { promptAction } from '@kit.ArkUI'
import { MqttQos } from '@ohos/mqtt';
import { getDeviceApi, InfoParams, MQTTTopics, putDeviceApi } from 'basic'
import MqttClientWrapper from 'basic/src/main/ets/utils/MyMqttClient'

// 定义 payload 的类型
interface MqttPayload {
  //发送的格式(因为接收是这个格式)
  BH?: number;
}

@Entry
@Component
struct BHView {
  @State BHValue: string = '0'
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

  @Builder
  descriptionBuilder() {
    Text('光照强度')
      .fontSize(14)
      .fontWeight(FontWeight.Medium)
      .width('100%')
      .height("100%")
      .textAlign(TextAlign.Center)
  }

  /**
   * 进入页面订阅并调用回调
   */
  async BH_MQTTInfoDB() {
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
          console.log(`[BHView] Received message on topic ${topic}: ${payload}`);
          if (res.data.data.topics === topic) {
            try {
              const data: MqttPayload = JSON.parse(payload);
              console.log('接收数据：', JSON.stringify(data));
              if (data.BH !== undefined) {
                this.BHValue = data.BH.toString()
              }
            } catch (parseError) {
              console.error(`[MQTT] JSON解析失败 (主题: ${topic}):`, parseError);
              console.error('原始payload:', payload);
              promptAction.openToast({message:'请检查数据格式'})
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

  build() {
    NavDestination() {
      Scroll() {
        Column({ space: 16 }) {
          // 设备图标展示
          Image(this.params.icon).height('46%').padding(14)

          Row() {
            Text('数据展示').height('100%').align(Alignment.Top)
              .fontWeight(FontWeight.Bold)
              .fontSize(20)
            Column() {
              Gauge({ value: Number(this.BHValue), min: 1, max: 5000 }) {
                Column() {
                  Text(this.BHValue)
                    .fontWeight(FontWeight.Medium)
                    .width('62%')
                    .maxFontSize("30.0vp")
                    .minFontSize("20.0vp")
                    .textAlign(TextAlign.Center)
                    .margin({ top: '35%' })
                    .textOverflow({ overflow: TextOverflow.Ellipsis })
                    .maxLines(1)
                }.width('100%').height('100%')
              }
              .height(200)
              .width(200)
              .startAngle(210)
              .endAngle(150)
              .colors([[new LinearGradient([{ color: "#ff000000", offset: 0 }, { color: "#ff606060", offset: 1 }]), 4],
                [new LinearGradient([{ color: "#ff757474", offset: 0 }, { color: "#ff9f9f9f", offset: 1 }]), 5],
                [new LinearGradient([{ color: "#ffa8a8a8", offset: 0 }, { color: "#ffc6c6c6", offset: 1 }]), 7],
                [new LinearGradient([{ color: "#ffd6d5d5", offset: 0 }, { color: "#ffe2e2e2", offset: 1 }]), 8],
                [new LinearGradient([{ color: "#ffe8e8e8", offset: 0 }, { color: "#fff3f2f2", offset: 1 }]), 5],
                [new LinearGradient([{ color: "#fff3f3f3", offset: 0 }, { color: "#ffffffff", offset: 1 }]), 4]])
              .strokeWidth(14)
              .description(this.descriptionBuilder)
              .trackShadow({ radius: 7, offsetX: 7, offsetY: 7 })
            }
          }
          .justifyContent(FlexAlign.SpaceBetween)
          .width('100%')
          .height(220)
          .backgroundColor($r('app.color.white'))
          .padding(16)
          .borderRadius(16)

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
                this.getUIContext().getFocusController().clearFocus(); //收起软键盘
                if (this.mqtt_info.topics.length > 0) {
                  console.log(`光照强度订阅主题: ${this.mqtt_info.topics}`);
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
          .height(170)
          .borderRadius(16)
          .padding(16)
          .backgroundColor($r('app.color.white'))
        }.padding(16).width('100%')
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
      const paramsArray = pathStack.getParamByName('BHView') as InfoParams[];
      if (paramsArray && paramsArray.length > 0) {
        const param = paramsArray[paramsArray.length - 1];
        this.params.deviceId = param.deviceId;
        this.params.deviceName = param.deviceName;
        this.params.icon = param.icon;
        this.params.linkMode = param.linkMode;
        console.log(`[DHTView] Received params: ${JSON.stringify(this.params)}`);
        await this.BH_MQTTInfoDB();
      }
    })
  }
}

export default BHView