import { Loading, MKNavPathStack, postMQTTApi, PreferenceManagerFactory, } from "basic";
import { promptAction } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";

const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')

interface api_mqtt_info {
  user_id: string
  host: string,
  client_id: string,
  username: string,
  password: string,
  connect_timeout: number,
  mqtt_version: number
}

@Entry
@Component
struct MQTTView {
  @State @Watch('onInput') mqtt_info: api_mqtt_info = {
    user_id: '',
    host: '',
    client_id: '',
    username: '',
    password: '',
    connect_timeout: 0,
    mqtt_version: 0
  }
  @State saveInfoButton: boolean = false //保存信息按钮开关
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '加载中...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })

  onInput() {
    this.saveInfoButton = this.mqtt_info.username && this.mqtt_info.host
      && this.mqtt_info.client_id && this.mqtt_info.password
      && this.mqtt_info.connect_timeout && this.mqtt_info.mqtt_version ? true : false
  }

  build() {
    NavDestination() {
      Column() {
        Scroll() {
          Column({ space: 16 }) {
            // 服务器地址、客户端ID
            Row() {
              Text('服务器地址：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ placeholder: 'tcp://IP:端口号', text: $$this.mqtt_info.host })
                .layoutWeight(2)
            }.width('100%')

            Row() {
              Text('客户端ID：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ placeholder: '不可重复', text: $$this.mqtt_info.client_id })
                .layoutWeight(2)
            }.width('100%')

            // 用户名、密码、超时、版本号
            Row() {
              Text('用户名：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ text: $$this.mqtt_info.username }).layoutWeight(2)
            }.width('100%')

            Row() {
              Text('密码：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ text: $$this.mqtt_info.password }).layoutWeight(2)
                .type(InputType.Password) // 设置为密码类型，隐藏输入内容
            }.width('100%')

            Row() {
              Text('超时时间：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ placeholder: '300', text: $$this.mqtt_info.connect_timeout })
                .layoutWeight(2)
            }.width('100%')

            Row() {
              Text('MQTT版本：').fontWeight(FontWeight.Medium).layoutWeight(1)
                .textAlign(TextAlign.Center)
              TextInput({ placeholder: '3 | 4 | 5', text: $$this.mqtt_info.mqtt_version })
                .layoutWeight(2)
            }.width('100%')

            // 保存按钮，保存所有配置信息到数据库
            Button('保存信息', { stateEffect: true }).width('90%').enabled(this.saveInfoButton)
              .onClick(async () => {
                this.dialog.open()
                this.getUIContext().getFocusController().clearFocus(); //收起软键盘
                const createResult = MqttClientWrapper.initialize({
                  serverUrl: this.mqtt_info.host,
                  clientId: this.mqtt_info.client_id,
                  persistenceType: 1 // 使用默认持久化方式
                });
                if (createResult.success) {
                  const connectResult = await MqttClientWrapper.connect({
                    userName: this.mqtt_info.username,
                    password: this.mqtt_info.password,
                    connectTimeout: Number(this.mqtt_info.connect_timeout),
                    MQTTVersion: Number(this.mqtt_info.mqtt_version)
                  });
                  if (connectResult.success) {
                    try {
                      const valueBucket: api_mqtt_info = {
                        user_id: userPref.getValue('userid'),
                        host: this.mqtt_info.host,
                        client_id: this.mqtt_info.client_id,
                        username: this.mqtt_info.username,
                        password: this.mqtt_info.password,
                        connect_timeout: this.mqtt_info.connect_timeout,
                        mqtt_version: this.mqtt_info.mqtt_version
                      };
                      const res = await postMQTTApi(valueBucket)
                      if (res.data.code === 201) {
                        userPref.setValue('mqtt_Flag', 'true')
                        MKNavPathStack.pop();
                        this.dialog.close() //关闭动画
                        promptAction.openToast({ message: '连接并保存成功' });
                      } else {
                        this.dialog.close() //关闭动画
                        promptAction.openToast({ message: '保存失败' });
                      }
                    } catch (error) {
                      this.dialog.close() //关闭动画
                      if (error.response) {
                        switch (error.response.status) {
                          case 400:
                            MqttClientWrapper.destroy()
                            promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                            break;
                          case 500:
                            MqttClientWrapper.destroy()
                            promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                            break;
                          case 404:
                            MqttClientWrapper.destroy()
                            promptAction.openToast({ message: '需有效token，请稍后重试' })
                            break;
                          default:
                            MqttClientWrapper.destroy()
                            promptAction.openToast({ message: '保存失败，请稍后重试' })
                            break;
                        }
                      }
                    }
                  } else {
                    MqttClientWrapper.destroy()
                    this.dialog.close()
                    promptAction.openToast({ message: '连接失败: ' + connectResult.error });
                  }
                } else {
                  MqttClientWrapper.destroy()
                  this.dialog.close()
                  promptAction.openToast({ message: '创建客户端失败: ' + createResult.error });
                }
              })
          }
          .width('100%')
          .borderRadius(20)
          .padding({
            top: 24,
            left: 14,
            right: 14,
            bottom: 24
          })
          .backgroundColor($r('app.color.white'))
          .shadow({
            //阴影凸显模块
            radius: 20,
            color: '#26000000',
            offsetY: 2
          })
        }
        .scrollBar(BarState.Off)
        .padding(12)
        .width('100%')
        .height('100%')
      }.width('100%').height('100%')
      .backgroundColor($r('app.color.common_gray_bg'))
    }
    // 页面导航配置
    .hideBackButton(true).title('MQTT配置')
  }
}

export default MQTTView
