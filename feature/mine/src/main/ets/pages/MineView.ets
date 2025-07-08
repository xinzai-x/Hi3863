import { Loading, MKNavPathStack, postLoginApi, PreferenceManagerFactory } from "basic";
import { promptAction } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";

const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');

export function updateUserInfo(): void {
  if (userPref.getValue('token') === '') {
    AppStorage.setOrCreate('avatar_url', 'https://i.postimg.cc/zfXht4P7/profile.png');
    AppStorage.setOrCreate('username', '请登录');
  } else {
    AppStorage.setOrCreate('avatar_url',
      userPref.getValue('avatar_url') || 'https://i.postimg.cc/zfXht4P7/profile.png');
    AppStorage.setOrCreate('username', userPref.getValue('username') || '请登录');
  }
}

@Entry
@Component
struct MineView {
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '登陆中 ...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })
  @StorageLink('avatar_url') avatarImage: string = 'https://i.postimg.cc/zfXht4P7/profile.png';
  @StorageLink('username') username: string = '请登录';
  @State isSheet: boolean = false

  aboutToAppear(): void {
    updateUserInfo()
  }

  build() {
    Navigation() {
      Column() {
        Scroll() {
          Column({ space: 16 }) {
            Row({ space: 10 }) {
              Row() {
                Image(this.avatarImage)
                  .width(60).height(60)
                  .borderRadius(35).backgroundColor($r('app.color.common_gray_01'))
              }.height('100%')

              Row() {
                Text(this.username)
                  .fontWeight(FontWeight.Medium)
                  .fontSize(16)
              }

              Blank()
              Image($r('app.media.ic_public_arrow_right')).height(26).fillColor($r('app.color.common_gray_01'))
            }
            .width('100%')
            .padding(16)
            .height(80)
            .borderRadius(16)
            .backgroundColor($r('app.color.white'))
            .shadow({
              radius: 20,
              color: '#26000000',
              offsetY: 2
            })
            .onClick(() => {
              if (userPref.getValue('token') === '') {
                MKNavPathStack.pushPathByName('LoginView', null);
              } else {
                MKNavPathStack.pushPathByName('UserInfoView', null);
              }
            })

            GridRow({ columns: 2 }) {
              GridCol() {
                NavComp({ icon: 'resources/base/media/ic_public_connection.svg', text: '连接信息' })
                  .onClick(() => {
                    MKNavPathStack.pushPathByName('DeviceLinkInfoView', null);
                  })
              }

              GridCol() {
                NavComp({ icon: 'https://i.postimg.cc/SNpDZvSY/customer.png', text: '智慧客服' })
                  .onClick(() => {
                    this.isSheet = true
                  })
                  .onClick(() => {
                    MKNavPathStack.pushPathByName('AIView', null);
                  })
              }
            }
            .width('100%')
            .backgroundColor($r('app.color.white'))
            .borderRadius(16)
            .shadow({
              radius: 20,
              color: '#26000000',
              offsetY: 2
            })

            Column() {
              List({ space: 10 }) {
                ListComp({ icon: 'resources/base/media/ic_public_settings.svg', text: '设置' })
                  .onClick(() => {
                    MKNavPathStack.pushPathByName('InstallView', null)
                  })
                ListComp({ icon: 'resources/base/media/help.svg', text: '帮助与反馈' })
                ListComp({ icon: 'resources/base/media/ic_public_quit.svg', text: '退出登陆' })
                  .onClick(() => {
                    if (userPref.getValue('token') !== '') {
                      this.getUIContext().showAlertDialog({
                        title: `退出登陆`,
                        message: "是否确定退出登陆",
                        buttons: [
                          {
                            value: '取消',
                            fontColor: $r('app.color.black'),
                            action: () => {
                              console.log('点击了取消');
                            }
                          },
                          {
                            value: '确定',
                            fontColor: $r('app.color.common_gray_02'),
                            action: async () => {
                              this.dialog.open()
                              try {
                                const res = await postLoginApi()
                                if (res.data.code === 200) {
                                  await userPref.deleteValue('token');
                                  await userPref.deleteValue('userid');
                                  await userPref.deleteValue('phone');
                                  await userPref.deleteValue('username');
                                  await userPref.deleteValue('avatar_url');
                                  await userPref.deleteValue('token_expire');
                                  promptAction.openToast({ message: '已退出登录' });
                                  AppStorage.setOrCreate('isLoggedIn', false); //将登陆状态置为关
                                  MqttClientWrapper.destroy() //销毁客户端
                                  updateUserInfo(); // 退出登录后立即更新 UI
                                  MKNavPathStack.pushPathByName('LoginView', null)
                                  console.log('点击了确定');
                                  this.dialog.close()
                                } else {
                                  promptAction.openToast({ message: res.data.message || '检查登录状态失败' });
                                  this.dialog.close()
                                }
                              } catch (error) {
                                if (error.response) {
                                  switch (error.response.status) {
                                    case 400:
                                      promptAction.openToast({ message: '缺少有效token' });
                                      break;
                                    case 500:
                                      promptAction.openToast({ message: '服务器内部错误，请稍后重试' });
                                      break;
                                  }
                                  this.dialog.close()
                                }
                              }
                            }
                          }
                        ]
                      });
                    }
                  })
              }
              .width('100%')
              .height('100%')
              .scrollBar(BarState.Off)
              .divider({
                strokeWidth: 1,
                startMargin: 42,
                endMargin: 6,
                color: '#ffdde3e3'
              })
            }
            .width('100%')
            .height('220')
            .backgroundColor($r('app.color.white'))
            .padding(16)
            .borderRadius(20)
            .shadow({
              radius: 20,
              color: '#26000000',
              offsetY: 2
            })
          }
          .padding({
            top: 24,
            bottom: 24,
            right: 16,
            left: 16
          })
        }
        .scrollBar(BarState.Off)
        .edgeEffect(EdgeEffect.Spring)
      }.height('100%').width('100%')
      .backgroundColor($r('app.color.common_gray_bg'))
    }
    .hideBackButton(true)
    .mode(NavigationMode.Stack)
    .title('我的')
    .titleMode(NavigationTitleMode.Mini)
    .hideToolBar(true)
  }
}

@Component
struct NavComp {
  icon: ResourceStr = '';
  text: string = '';

  build() {
    GridCol() {
      Column({ space: 2 }) {
        Image(this.icon)
          .width(34).height(34)
          .aspectRatio(1)
        Text(this.text)
          .fontWeight(FontWeight.Medium)
          .fontSize(12)
      }
      .padding({ top: 10, bottom: 10 })
    }
  }
}

@Component
struct ListComp {
  icon: ResourceStr = '';
  text: string = '';

  build() {
    ListItem() {
      Row({ space: 14 }) {
        Image(this.icon)
          .height(32).width(32)
        Text(this.text).fontWeight(FontWeight.Medium).fontSize(14)

        Blank()
        Image($r('app.media.ic_public_arrow_right')).height(20).fillColor($r('app.color.common_gray_01'))
      }
      .height(54)
      .width('100%')
      .alignItems(VerticalAlign.Center)
    }
  }
}

export default MineView;
