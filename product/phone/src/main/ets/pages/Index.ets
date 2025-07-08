import { getMQTTApi, Loading, MKNavPathStack, PreferenceManagerFactory } from 'basic'
import DeviceView from 'device/src/main/ets/pages/DeviceView'
import MineView, { updateUserInfo } from 'mine/src/main/ets/pages/MineView'
import { promptAction } from '@kit.ArkUI'
import MqttClientWrapper from 'basic/src/main/ets/utils/MyMqttClient'

interface TabItem {
  text: string
  active: ResourceStr
}

@Entry
@Component
struct Index {
  @State activeIndex: number = 0
  @State isShowSheet: boolean = false
  @StorageLink('popInfo')
  popUIState: Record<string, boolean> = {}
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '数据加载中...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })
  list: TabItem[] = [//工具栏tab
    { text: '设备', active: $r('app.media.ic_public_home_filled') },
    { text: '我的', active: $r('app.media.ic_user_portrait_fill') },
  ]

  // 检查是否有效token
  async inspectToken(): Promise<void> {
    this.dialog.open()
    const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');
    // const userPhone: string = userPref.getValue('phone')
    const token: string = userPref.getValue('token') || '';
    const tokenExpire: string = userPref.getValue('token_expire') || '';
    console.log('token和tokenExpire', token + "" + tokenExpire.toString())
    if (token.trim() !== '' && token !== null && tokenExpire.toString() !== '') {
      const now = new Date(); // 当前时间
      const expireDate = new Date(tokenExpire); // 转为 Date 对象
      const thresholdDate = new Date(now.getTime() + 30 * 24 * 60 * 60 * 1000); // 当前时间 + 30 天
      if (expireDate <= thresholdDate) {
        // 本地检查 token 是否过期
        console.log('Token 有效，保持登录');
        AppStorage.setOrCreate('isLoggedIn', true);
        updateUserInfo();
        userPref.setValue('token_expire', new Date().toString()) //更新本地token有效期
        console.log('有效期：', tokenExpire);
        try {
          const res = await getMQTTApi(userPref.getValue('userid'))
          // 进入页面连接mqtt
          if (res.data.code === 200) {
            const createResult = MqttClientWrapper.initialize({
              serverUrl: res.data.data.host,
              clientId: res.data.data.client_id,
              persistenceType: 1 // 使用默认持久化方式
            });
            if (createResult.success) {
              const connectResult = await MqttClientWrapper.connect({
                userName: res.data.data.username,
                password: res.data.data.password,
                connectTimeout: Number(res.data.data.connect_timeout),
                MQTTVersion: Number(res.data.data.mqtt_version)
              });
              if (connectResult.success) {
                userPref.setValue('mqtt_Flag', 'true')
                promptAction.openToast({ message: 'MQTT连接成功' });
              } else {
                userPref.setValue('mqtt_Flag', 'true')
                promptAction.openToast({ message: 'MQTT连接失败' });
              }
              this.dialog.close()
            } else {
              promptAction.openToast({ message: '创建客户端失败: ' + createResult.error });
              this.dialog.close()
            }
          }
        } catch (error) {
          if (error.response) {
            switch (error.response.status) {
              case 401:
                this.dialog.close()
                await userPref.deleteValue('token');
                await userPref.deleteValue('phone');
                await userPref.deleteValue('username');
                await userPref.deleteValue('avatar_url');
                await userPref.deleteValue('token_expire');
                AppStorage.setOrCreate('username', '请重新登录');
                AppStorage.setOrCreate('avatar_url', 'https://i.postimg.cc/zfXht4P7/profile.png');
                AppStorage.setOrCreate('isLoggedIn', false);
                updateUserInfo();
                MKNavPathStack.pushPathByName('LoginView', null);
                break;
              case 404:
                this.dialog.close()
                promptAction.openToast({ message: '未找到mqtt配置' })
                break;
              case 403:
                this.dialog.close()
                promptAction.openToast({ message: '无权访问' })
                break;
              case 500:
                this.dialog.close()
                promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                break;
              default:
                this.dialog.close()
                promptAction.openToast({ message: '查询失败，请稍后重试' })
                break;
            }
          }
        }
        return; // 停留在当前页面
      } else {
        // Token 缺失或过期，清空存储并跳转登录
        console.log('Token 无效或过期，跳转登录');
        promptAction.openToast({ message: '请登录' });
        await userPref.deleteValue('token');
        await userPref.deleteValue('phone');
        await userPref.deleteValue('username');
        await userPref.deleteValue('avatar_url');
        await userPref.deleteValue('token_expire');
        AppStorage.setOrCreate('username', '请登录');
        AppStorage.setOrCreate('avatar_url', 'https://i.postimg.cc/zfXht4P7/profile.png');
        AppStorage.setOrCreate('isLoggedIn', false);
        updateUserInfo();
        MKNavPathStack.pushPathByName('LoginView', null);
        this.dialog.close()
      }
    } else {
      // Token 缺失或过期，清空存储并跳转登录
      console.log('Token 无效或过期，跳转登录');
      promptAction.openToast({ message: '请登录' });
      await userPref.deleteValue('token');
      await userPref.deleteValue('phone');
      await userPref.deleteValue('username');
      await userPref.deleteValue('avatar_url');
      await userPref.deleteValue('token_expire');
      AppStorage.setOrCreate('username', '请登录');
      AppStorage.setOrCreate('avatar_url', 'https://i.postimg.cc/zfXht4P7/profile.png');
      AppStorage.setOrCreate('isLoggedIn', false);
      updateUserInfo();
      this.dialog.close()
      MKNavPathStack.pushPathByName('LoginView', null);
    }
  }

  aboutToAppear() {
    // const context = getContext()
    // AppStorage.setOrCreate('context', context)
    this.inspectToken();
  }

  build() {
    // Navigation 作为根路由组件，传入页面栈对象
    // MKNavPathStack 抽离成公共层，这样共享同一个页面栈
    Navigation(MKNavPathStack) {
      Tabs({ barPosition: BarPosition.End }) {
        ForEach(this.list, (item: TabItem, index: number) => {
          TabContent() {
            if (index == 0) {
              DeviceView()
            } else if (index == 1) {
              MineView()
            }
          }
          .tabBar(this.TabItemBuilder(item, index))
        })
      }.scrollable(false)
      .onTabBarClick(index => {
        this.activeIndex = index
      })
    }
    .hideTitleBar(true) // 隐藏默认的标题栏
    .mode(NavigationMode.Stack) // 设置为单栏模式，默认时 Auto 自动分栏
  }

  //底部工具栏信息
  @Builder
  TabItemBuilder(item: TabItem, index: number) {
    Column() {
      if (this.activeIndex === index) {
        Image(item.active)
          .width(24).fillColor(Color.Black)
          .aspectRatio(1)
        Text(item.text)
          .fontWeight(FontWeight.Medium)
          .fontSize(12)
      } else {
        Image(item.active)
          .width(24).fillColor(Color.Gray)
          .aspectRatio(1)
        Text(item.text)
          .fontColor(Color.Gray)
          .fontSize(12)
      }
    }
    .justifyContent(FlexAlign.SpaceEvenly)
    .height(50)
  }
}
