import { InfoParams, getDeviceApi, MQTTTopics, putDeviceApi } from "basic";
import { promptAction, SegmentButtonV2Items, CapsuleSegmentButtonV2 } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";
import { MqttQos } from "@ohos/mqtt";

@Entry
@Component
struct LEDView {
  @State isRefreshing: boolean = false; //下拉刷新开关
  //保存设备基础信息
  @State params: InfoParams = {
    deviceId: '',
    deviceName: '', // 设备名称
    linkMode: '', // 连接方式(MQTT/HTTP)
    icon: '', // 设备图标
  };
  //保存MQTT配置信息
  @State mqtt_info: MQTTTopics = {
    topics: '', // 订阅主题列表
    qos: 0 // 初始化为 0（对应 AT_MOST_ONCE）
  };
  @State textSelectedIndex: number = 0; //rgb选择
  // RGB灯亮度调节变量
  @State RGB_PWM: number = 45;
  @State RGB_Select: number = 0;
  @State EnableInc: boolean = true
  @State EnableDec: boolean = true

  build() {
    NavDestination() {
      Scroll() {
        Column({ space: 16 }) {
          // 设备图标展示
          Image(this.params.icon).height('46%').padding(14)

          Column({ space: 14 }) {
            Row() {
              Text('发送主题：').fontWeight(FontWeight.Medium)
              TextArea({ text: $$this.mqtt_info.topics }).layoutWeight(1)
            }.width('100%')

            // 发送按钮
            Button('保存').width('80%').fontWeight(FontWeight.Medium)
              .onClick(async () => {
                this.getUIContext().getFocusController().clearFocus(); //收起软键盘
                if (!this.mqtt_info.topics) {
                  promptAction.openToast({ message: '请输入发送主题' });
                  return;
                }
                try {
                  const res = await putDeviceApi(this.params.deviceId, this.mqtt_info)
                  if (res.data.code === 200) {
                    promptAction.openToast({ message: "保存成功" })
                  }
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 404:
                        promptAction.openToast({ message: '设备未找到' })
                        break;
                      case 401:
                        promptAction.openToast({ message: '无有效token' })
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
              })
          }
          .width('100%')
          .backgroundColor($r('app.color.white'))
          .height(120)
          .borderRadius(16)
          .padding(16)

          //led开关模块
          Row() {
            Text(`LED灯开关`).fontSize(16).fontWeight(FontWeight.Medium)
            Toggle({ type: ToggleType.Switch, isOn: false })
              .onChange(async (isOn: boolean) => {
                if (isOn === true) {
                  const ledData = JSON.stringify({ led: "on" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  // 设备名即为发送主题
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, ledData, qos)
                  promptAction.openToast({
                    message: result?.success ? '消息发送成功' : `发送失败: ${result?.error}`
                  });
                } else if (isOn === false) {
                  const ledData = JSON.stringify({ led: "off" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, ledData, qos)
                  promptAction.openToast({
                    message: result?.success ? '消息发送成功' : `发送失败: ${result?.error}`
                  });
                }
              })
          }
          .padding(16)
          .width('100%')
          .height(70)
          .justifyContent(FlexAlign.SpaceBetween)
          .backgroundColor($r('app.color.white'))
          .borderRadius(16)

          Column({ space: 16 }) {
            Text(`RGB灯`).fontSize(16).fontWeight(FontWeight.Medium)
              .width('100%').textAlign(TextAlign.Start)
            // 开启RGB
            CapsuleSegmentButtonV2({
              items: new SegmentButtonV2Items([
                { text: '关闭' },
                { text: '红色' },
                { text: '绿色' },
                { text: '蓝色' }
              ]),
              selectedIndex: this.textSelectedIndex!!,
              onItemClicked: async (index: number) => {
                if (index === 0) {
                  const rgbData = JSON.stringify({ "led": "rgboff" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, rgbData, qos)
                  promptAction.openToast({
                    message: result?.success ? '关灯成功' : `关灯失败: ${result?.error}`
                  });
                } else if (index === 1) {
                  this.RGB_Select = 1
                  const rgbData = JSON.stringify({ "led": "redon" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, rgbData, qos)
                  promptAction.openToast({
                    message: result?.success ? '开灯成功' : `开灯失败: ${result?.error}`
                  });
                } else if (index === 2) {
                  this.RGB_Select = 2
                  const rgbData = JSON.stringify({ "led": "greenon" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, rgbData, qos)
                  promptAction.openToast({
                    message: result?.success ? '开灯成功' : `开灯失败: ${result?.error}`
                  });
                } else if (index === 3) {
                  this.RGB_Select = 3
                  const rgbData = JSON.stringify({ "led": "blueon" });
                  const qos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                  const result = await MqttClientWrapper.publish(this.mqtt_info.topics, rgbData, qos)
                  promptAction.openToast({
                    message: result?.success ? '开灯成功' : `开灯失败: ${result?.error}`
                  });
                }
              }
            })
            // 调节RGB亮度
            Row() {
              Text(`亮度`).fontSize(16).fontWeight(FontWeight.Medium)
              Counter() {
                Text(this.RGB_PWM.toString())
              }.enableDec(this.EnableDec).enableInc(this.EnableInc)
              .onInc(async () => {
                if (this.RGB_PWM >= 255) {
                  this.EnableInc = false
                } else {
                  this.EnableInc = true
                  this.EnableDec = true
                  this.RGB_PWM += 15;
                  switch (this.RGB_Select) {
                    case 1:
                      const redPWM = JSON.stringify({ "redpwm": this.RGB_PWM });
                      const redQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, redPWM, redQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                    case 2:
                      const greenPWM = JSON.stringify({ "greenpwm": this.RGB_PWM });
                      const greenQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, greenPWM, greenQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                    case 3:
                      const bluePWM = JSON.stringify({ "bluepwm": this.RGB_PWM });
                      const blueQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, bluePWM, blueQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                  }
                }
              })
              .onDec(async () => {
                if (this.RGB_PWM <= 0) {
                  this.EnableDec = false
                } else {
                  this.EnableInc = true
                  this.EnableDec = true
                  this.RGB_PWM -= 15;
                  switch (this.RGB_Select) {
                    case 0:
                      const redPWM = JSON.stringify({ "redpwm": this.RGB_PWM });
                      const redQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, redPWM, redQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                    case 1:
                      const greenPWM = JSON.stringify({ "greenpwm": this.RGB_PWM });
                      const greenQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, greenPWM, greenQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                    case 2:
                      const bluePWM = JSON.stringify({ "bluepwm": this.RGB_PWM });
                      const blueQos = [0, 1, 2].includes(this.mqtt_info.qos) ? this.mqtt_info.qos as MqttQos : 0;
                      setTimeout(async () => {
                        const result = await MqttClientWrapper.publish(this.mqtt_info.topics, bluePWM, blueQos)
                        promptAction.openToast({
                          message: result?.success ? '调节成功' : `调节失败: ${result?.error}`
                        });
                      }, 500)
                      break
                  }
                }
              })
            }.width('100%').justifyContent(FlexAlign.SpaceBetween)
          }
          .padding(16)
          .width('100%')
          .height(160)
          .backgroundColor($r('app.color.white'))
          .borderRadius(16)
        }.width('100%').padding(16)
      }.height('100%').scrollBar(BarState.Off).edgeEffect(EdgeEffect.Spring)
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
    }.title(this.params.deviceName).backgroundColor($r('app.color.common_gray_bg'))
    .onReady(async (context: NavDestinationContext) => {
      const pathStack = context.pathStack;
      const paramsArray = pathStack.getParamByName('LEDInfoView') as InfoParams[];
      if (paramsArray && paramsArray.length > 0) {
        const param = paramsArray[paramsArray.length - 1];
        this.params.deviceId = param.deviceId;
        this.params.deviceName = param.deviceName;
        this.params.icon = param.icon;
        this.params.linkMode = param.linkMode;
        console.log(`[LEDView] Received params: ${JSON.stringify(this.params)}`);
        const res = await getDeviceApi(this.params.deviceId)
        if (res.data.code === 200 && res.data.data.topics !== null) {
          this.mqtt_info.topics = res.data.data.topics
        } else {
          promptAction.openToast({ message: '请先填写发送主题并保存' })
        }
      }
    })
  }
}

export default LEDView