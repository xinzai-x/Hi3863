import { getMQTTApi, Loading, mqttInfo, PreferenceManagerFactory, putMQTTApi } from "basic";
import { promptAction } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";

const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');

interface host {
  host: string
}

interface client_id {
  client_id: string
}

interface username {
  username: string
}

interface password {
  password: string
}

interface connect_timeout {
  connect_timeout: string
}

interface mqtt_version {
  mqtt_version: string
}

@Entry
@Component
struct DeviceLinkInfoView {
  @State message: string = '设备连接页面';
  @State changInfo: string = ''
  @State titleName: string = ''
  @State isBindSheet: boolean = false
  @State isHTTPSheet: boolean = false
  @State isChangInfo: boolean = false
  @State mqtt_Info: mqttInfo = {
    id: '',
    host: '',
    client_id: '',
    username: '',
    password: '',
    connect_timeout: 0,
    mqtt_version: 0
  }
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '修改信息中' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })

  // 修改MQTT信息
  @Builder
  ChangeMQTTInfo(name: string) {
    Column({ space: 20 }) {
      TextInput({ placeholder: '请输入信息', text: $$this.changInfo }).width('90%')
      Button('设置信息').width('80%')
        .onClick(async () => {
          switch (name) {
            case '服务器地址':
              this.dialog.open()
              this.mqtt_Info.host = this.changInfo
              const flagHost = await VerifiedLink(this.mqtt_Info)
              if (flagHost === true) {
                try {
                  const change: host = {
                    host: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        this.dialog.close()
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        this.dialog.close()
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 500:
                        this.dialog.close()
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break

            case '客户端ID':
              this.dialog.open()
              this.mqtt_Info.client_id = this.changInfo
              const flag_Client_Id = await VerifiedLink(this.mqtt_Info)
              if (flag_Client_Id === true) {
                try {
                  const change: client_id = {
                    client_id: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        this.dialog.close()
                        break;
                      case 401:
                        promptAction.openToast({ message: '无有效token' })
                        this.dialog.close()
                        break;
                      case 500:
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        this.dialog.close()
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break
            case '用户名':
              this.dialog.open()
              this.mqtt_Info.username = this.changInfo
              const flagUsername = await VerifiedLink(this.mqtt_Info)
              if (flagUsername === true) {
                try {
                  const change: username = {
                    username: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        this.dialog.close()
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        this.dialog.close()
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 500:
                        this.dialog.close()
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break
            case '密码':
              this.dialog.open()
              this.mqtt_Info.password = this.changInfo
              const flagPassword = await VerifiedLink(this.mqtt_Info)
              if (flagPassword === true) {
                try {
                  const change: password = {
                    password: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        this.dialog.close()
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        this.dialog.close()
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 500:
                        this.dialog.close()
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break
            case '超时时间':
              this.dialog.open()
              this.mqtt_Info.connect_timeout = Number(this.changInfo)
              const flagConnect_timeout = await VerifiedLink(this.mqtt_Info)
              if (flagConnect_timeout === true) {
                try {
                  const change: connect_timeout = {
                    connect_timeout: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        this.dialog.close()
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        this.dialog.close()
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 500:
                        this.dialog.close()
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break
            case 'MQTT版本':
              this.dialog.open()
              this.mqtt_Info.mqtt_version = Number(this.changInfo)
              const flagMqtt = await VerifiedLink(this.mqtt_Info)
              if (flagMqtt === true) {
                try {
                  const change: mqtt_version = {
                    mqtt_version: this.changInfo
                  }
                  const res = await putMQTTApi(userPref.getValue('userid'), change)
                  if (res.data.code === 200) {
                    this.isBindSheet = false
                    this.isChangInfo = false
                    promptAction.openToast({ message: '保存服务器地址信息成功' })
                  }
                  this.dialog.close()
                } catch (error) {
                  if (error.response) {
                    switch (error.response.status) {
                      case 400:
                        this.dialog.close()
                        promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                        break;
                      case 401:
                        this.dialog.close()
                        promptAction.openToast({ message: '无有效token' })
                        break;
                      case 500:
                        this.dialog.close()
                        promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                        break;
                      case 404:
                        this.dialog.close()
                        promptAction.openToast({ message: '未找到配置' })
                        break;
                      default:
                        this.dialog.close()
                        promptAction.openToast({ message: '保存失败，请稍后重试' })
                        break;
                    }
                  }
                }
              } else {
                this.dialog.close()
                promptAction.openToast({ message: 'MQTT连接不上，请检查' })
              }
              break
            default:
              this.dialog.close()
              promptAction.openToast({ message: '无效信息' })
              break
          }
        })
    }.height('100%').width('100%').justifyContent(FlexAlign.Center)
  }

  // MQTT连接信息半模态展示页面
  @Builder
  MQTTInfoView() {
    Column({ space: 16 }) {
      List({ space: 30 }) {
        ListItem() {
          Row() {
            Text('服务器地址').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text(this.mqtt_Info.host).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect() //断开连接
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = '服务器地址'
          this.isChangInfo = true
        })

        ListItem() {
          Row() {
            Text('客户端ID').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text(this.mqtt_Info.client_id).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect()
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = '客户端ID'
          this.isChangInfo = true
        })

        ListItem() {
          Row() {
            Text('用户名').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text(this.mqtt_Info.username).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect()
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = '用户名'
          this.isChangInfo = true
        })

        ListItem() {
          Row() {
            Text('密码').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text('******').fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect()
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = '密码'
          this.isChangInfo = true
        })

        ListItem() {
          Row() {
            Text('超时时间').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text(this.mqtt_Info.connect_timeout.toString())
              .fontWeight(FontWeight.Medium)
              .fontSize(16)
              .margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect()
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = '超时时间'
          this.isChangInfo = true
        })

        ListItem() {
          Row() {
            Text('MQTT版本').fontWeight(FontWeight.Medium).fontSize(16)
              .margin({ left: 16 })
            Blank();
            Text(this.mqtt_Info.mqtt_version.toString()).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 1 })
              .fontColor($r('app.color.common_gray_01'))
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          await MqttClientWrapper.disconnect()
          await MqttClientWrapper.destroy() //销毁mqtt客户端
          this.titleName = 'MQTT版本'
          this.isChangInfo = true
        })
      }
      .width('100%')
      .height('100%')
      .backgroundColor($r('app.color.white'))
      .padding({ top: 20, bottom: 20 })
      .divider({
        strokeWidth: 1, //分割线宽度
        color: '#ffe9f0f0'   //分隔线颜色
      })
    }.height('100%').width('100%')
    .bindSheet(this.isChangInfo, this.ChangeMQTTInfo(this.titleName),
      {
        detents: [SheetSize.MEDIUM],
        preferType: SheetType.POPUP,
        title: { title: `设置${this.titleName}` },
        onWillDismiss: ((DismissSheetAction: DismissSheetAction) => {
          if (DismissSheetAction.reason == DismissReason.CLOSE_BUTTON) {
            this.isChangInfo = false
            DismissSheetAction.dismiss(); //注册dismiss行为
          }
        }),
      })
  }

  @Builder
  HTTPInfoView() {
    Column() {
      Text('HTTP信息展示')
    }
  }

  build() {
    Column() {
      List({ space: 26 }) {
        ListItem() {
          Row() {
            Text("MQTT信息").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
            Blank();
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%')
        }.onClick(async () => {
          this.isBindSheet = true
          try {
            const res = await getMQTTApi(userPref.getValue('userid'))
            if (res.data.code === 200) {
              this.mqtt_Info.host = res.data.data.host
              this.mqtt_Info.client_id = res.data.data.client_id
              this.mqtt_Info.username = res.data.data.username
              this.mqtt_Info.password = res.data.data.password
              this.mqtt_Info.connect_timeout = res.data.data.connect_timeout
              this.mqtt_Info.mqtt_version = res.data.data.mqtt_version
            }
          } catch (error) {
            console.error('查询错误：', error);
            if (error.response) {
              switch (error.response.status) {
                case 401:
                  promptAction.openToast({ message: '无有效token' })
                  break;
                case 403:
                  promptAction.openToast({ message: '无权限访问' })
                  break;
                case 404:
                  promptAction.openToast({ message: '未找到mqtt配置信息' })
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
        .bindSheet(this.isBindSheet, this.MQTTInfoView(),
          {
            detents: [SheetSize.MEDIUM],
            preferType: SheetType.POPUP,
            title: { title: 'MQTT配置信息' },
            onWillDismiss: ((DismissSheetAction: DismissSheetAction) => {
              if (DismissSheetAction.reason == DismissReason.CLOSE_BUTTON) {
                this.isBindSheet = false
                DismissSheetAction.dismiss(); //注册dismiss行为
              }
            }),
          })

        ListItem() {
          Row() {
            Text("HTTP信息").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
            Blank();
            Image($r('app.media.ic_public_arrow_right'))
              .height(26)
              .fillColor($r('app.color.common_gray_01'))
              .margin({ right: 10 })
          }.width('100%').onClick(() => {
            this.isHTTPSheet = true
          })
          .bindSheet(this.isHTTPSheet, this.HTTPInfoView(),
            {
              detents: [SheetSize.MEDIUM],
              preferType: SheetType.POPUP,
              title: { title: 'HTTP配置信息' },
              onWillDismiss: ((DismissSheetAction: DismissSheetAction) => {
                if (DismissSheetAction.reason == DismissReason.CLOSE_BUTTON) {
                  this.isHTTPSheet = false
                  DismissSheetAction.dismiss(); //注册dismiss行为
                }
              }),
            })
        }
      }.width('100%').height('100%')
      .divider({
        strokeWidth: 1, //分割线宽度
        color: '#ffe9f0f0'   //分隔线颜色
      })
    }.padding({ top: 16 })
    .width('100%')
    .height('100%')
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM])
  }
}

// 验证修改后MQTT是否可用
async function VerifiedLink(mqttInfo: mqttInfo) {
  console.log(`传进信息：${JSON.stringify(mqttInfo)}`)
  const createResult = MqttClientWrapper.initialize({
    serverUrl: mqttInfo.host,
    clientId: mqttInfo.client_id,
    persistenceType: 1 // 使用默认持久化方式
  });
  if (createResult.success) {
    const connectResult = await MqttClientWrapper.connect({
      userName: mqttInfo.username,
      password: mqttInfo.password,
      connectTimeout: Number(mqttInfo.connect_timeout),
      MQTTVersion: Number(mqttInfo.mqtt_version)
    });
    if (connectResult.success) {
      console.log('mqtt连接成功')
      return true
    } else {
      console.log('mqtt连接失败')
      await MqttClientWrapper.disconnect() //断开连接
      await MqttClientWrapper.destroy() //销毁mqtt客户端
      return false
    }
  } else {
    console.log('创建mqtt客户端失败')
    await MqttClientWrapper.disconnect() //断开连接
    await MqttClientWrapper.destroy() //销毁mqtt客户端
    return false
  }
}

export default DeviceLinkInfoView
