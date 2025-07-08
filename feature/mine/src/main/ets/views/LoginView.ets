import { promptAction } from "@kit.ArkUI"
import {
  DeviceStateManager,
  getLoginApi,
  getMQTTApi,
  huaweiAuthPlugin,
  Loading,
  MKNavPathStack,
  PreferenceManagerFactory
} from "basic"
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient"


@Entry
@Component
struct LoginView {
  @State @Watch('onInput') phone: string = ''
  @State @Watch('onInput') password: string = ''
  @State valid: boolean = false //控制按钮颜色及状态
  @State agree: boolean = false //是否勾选协议
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '登陆中 ...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })

  onInput() {
    this.valid = this.phone && this.password ? true : false
  }

  //按钮点击调用函数
  async onSubmit() {
    const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo')
    if (!this.agree) {
      promptAction.openToast({ message: '请先勾选协议' })
      return;
    }
    this.dialog.open()
    try {
      const res = await getLoginApi(this.phone, this.password)
      console.log('登录状态码:', res.data.code.toString());
      if (res.data.code === 200) {
        // 将信息保存到首选项中
        await userPref.setValue('userid', res.data.data.id)
        await userPref.setValue('token', res.data.data.token)
        await userPref.setValue('phone', res.data.data.phone)
        await userPref.setValue('token_expire', res.data.data.token_expire.toString())
        await userPref.setValue('username', res.data.data.username)
        await userPref.setValue('avatar_url', res.data.data.avatar_url)
        await userPref.setValue('mqtt_Flag', 'false')
        await DeviceStateManager.loadDeviceList()
        // 全局存储用户名和头像
        AppStorage.setOrCreate('username', res.data.data.username);
        AppStorage.setOrCreate('avatar_url', res.data.data.avatar_url);
        AppStorage.setOrCreate('isLoggedIn', true);
        PersistentStorage.persistProp('isLoggedIn', true);
        try {
          // 若有，连接mqtt
          const res1 = await getMQTTApi(res.data.data.id)
          if (res1.data.code === 200 && res1.data.data.host !== null) {
            console.log("账户存在mqtt")
            const createResult = MqttClientWrapper.initialize({
              serverUrl: res1.data.data.host,
              clientId: res1.data.data.client_id,
              persistenceType: 1 // 使用默认持久化方式
            });
            if (createResult.success) {
              console.log("成功创建客户端")
              const connectResult = await MqttClientWrapper.connect({
                userName: res1.data.data.username,
                password: res1.data.data.password,
                connectTimeout: Number(res1.data.data.connect_timeout),
                MQTTVersion: Number(res1.data.data.mqtt_version)
              });
              if (connectResult.success) {
                console.log("连接mqtt成功")
                await userPref.setValue('mqtt_Flag', 'true')
                this.dialog.close()
                promptAction.openToast({ message: '登录成功并MQTT连接成功' })
                MKNavPathStack.clear();
              } else {
                console.log("连接mqtt成功")
                this.dialog.close()
                promptAction.openToast({ message: '登录成功但MQTT连接不成功' })
                MKNavPathStack.clear();
              }
            }
          }
        } catch (error) {
          console.error('获取mqtt信息错误：', error);
          this.dialog.close() //关闭动画

          if (error.response) {
            switch (error.response.status) {
              case 403:
                promptAction.openToast({ message: '无权访问，请检查输入信息' })
                break;
              case 401:
                promptAction.openToast({ message: '无有效token' })
                break;
              case 404:
                promptAction.openToast({ message: '登录成功' })
                MKNavPathStack.pop();
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
      }
    } catch (error) {
      console.error('登录错误：', error);
      this.dialog.close() //关闭动画

      if (error.response) {
        switch (error.response.status) {
          case 400:
            promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
            break;
          case 401:
            promptAction.openToast({ message: '无效手机号或密码，请检查输入信息' })
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
  }

  build() {
    // 登录表单
    Scroll() {
      Column() {
        Column({ space: 16 }) {
          Text('账号密码登录')
            .fontSize(24)
            .width('100%')
            .fontWeight(500)
            .margin({ bottom: 10 })
          TextInput({ placeholder: '请输入手机号', text: $$this.phone })
            .placeholderFont({ size: 14 }).height(48)
          TextInput({ placeholder: '请输入密码', text: $$this.password })
            .type(InputType.Password).height(48)
            .placeholderFont({ size: 14 })

          Row({ space: 4 }) {
            Checkbox()
              .select($$this.agree)
              .width(12)
              .aspectRatio(1)
              .mark({ size: 10, strokeWidth: 1 })
            Text() {
              Span('查看并同意')
              Span('《隐私条款》')
                .fontColor('#007DFB')
              Span('和')
              Span('《用户协议》')
                .fontColor('#007DFB')
            }.fontWeight(FontWeight.Medium)
            .width('100%')
            .fontSize(12)
          }

          Button('登录', { stateEffect: true }).enabled(this.valid)
            .backgroundColor(this.valid ? $r('app.color.black') : $r('app.color.gray'))// .formStyle()
            .width('90%')
            .onClick(() => {
              this.getUIContext().getFocusController().clearFocus(); //收起软键盘
              this.onSubmit()
            })

          Row() {
            Row() {
              Text('注册账号')
                .fontWeight(FontWeight.Medium)
                .fontColor('#007DFB')
                .fontSize(14)
            }.width(60).height(30)
            .onClick(() => {
              MKNavPathStack.pushPathByName('RegisterView', null)
            })

            Row() {
              Text('忘记密码').fontWeight(FontWeight.Medium)
                .fontColor('#007DFB')
                .fontSize(14)
            }.width(60).height(30)
            .onClick(() => {
              MKNavPathStack.pushPathByName('ChangePassWordView', null)
            })
          }.justifyContent(FlexAlign.SpaceBetween)
          .width('100%')
        }

        Column({ space: 12 }) {
          Stack() {
            Text()
              .width(200)
              .height(1)
            Text('其他登录方式')
              .width(100)
              .fontSize(14).fontWeight(FontWeight.Medium)
              .textAlign(TextAlign.Center)
          }

          Row() {
            Button('华为登录', { type: ButtonType.Normal, stateEffect: true })
              .borderRadius(10)
              .width('66%')
              .backgroundColor('#FF133D')
              .fontWeight(FontWeight.Medium)
              .onClick(async () => {
                const code = await huaweiAuthPlugin.requestAuth()
                // 可以打印一下结果测试
                console.log(`code:${code}`)
                promptAction.openToast({ message: `code:${code}` })
              })
          }
        }
      }.width('100%').padding(30).height('100%')
      .justifyContent(FlexAlign.SpaceAround)
    }.width('100%').height('100%')
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
  }
}

export default LoginView