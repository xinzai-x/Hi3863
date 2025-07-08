import { deleteRegisterApi, getRegisterApi, Loading, MKNavPathStack, PreferenceManagerFactory } from "basic";
import { promptAction } from "@kit.ArkUI";
import MqttClientWrapper from "basic/src/main/ets/utils/MyMqttClient";

const userPref = PreferenceManagerFactory.getInstance().getPreference('userInfo');

@Entry
@Component
struct UserInfoView {
  //加载动画组件
  dialog = new CustomDialogController({
    builder: Loading({ message: '登陆中 ...' }),
    customStyle: true, // 自定义样式
    alignment: DialogAlignment.Center, // 居中
    autoCancel: false //禁止点击其它区域关闭加载弹出
  })
  @StorageLink('username') username: string = '请登录';
  @StorageLink('gender') gender: string = '未知';
  @StorageLink('age') age: number = 0;
  @StorageLink('phone') phone: string = '未知';
  @StorageLink('birth_date') birth_date: Date = new Date;

  //通过API获取用户信息
  async getUserInfo() {
    const res = await getRegisterApi(userPref.getValue('phone'))
    this.username = res.data.data.username
    this.gender = res.data.data.gender
    this.age = res.data.data.age
    this.phone = res.data.data.phone
    this.birth_date = res.data.data.birth_date
    console.log('生日:', this.birth_date)
  }

  // 在组件内添加方法
  private formatDate(date: Date | string): string {
    // 如果 date 是字符串（如 API 返回的 ISO 格式），先转为 Date 对象
    const d = typeof date === 'string' ? new Date(date) : date;

    // 返回本地化的日期格式（如 "2006-06-15"）
    return `${d.getFullYear()}-${(d.getMonth() + 1).toString().padStart(2, '0')}-${d.getDate()
      .toString()
      .padStart(2, '0')}`;
  }

  aboutToAppear(): void {
    this.getUserInfo()
  }

  build() {
    Scroll() {
      Column({ space: 16 }) {
        Image(userPref.getValue('avatar_url'))
          .height(200)
          .width(200)
          .backgroundColor($r('app.color.common_gray_01'))
          .borderRadius(120)
          .shadow({
            //阴影凸显模块
            radius: 20,
            color: '#26000000',
            offsetY: 2
          })

        List({ space: 26 }) {
          ListItem() {
            Row() {
              Text("用户名").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
              Blank();
              Text(this.username).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 2 })
                .fontColor($r('app.color.common_gray_01'))
              Image($r('app.media.ic_public_arrow_right'))
                .height(26)
                .fillColor($r('app.color.common_gray_01'))
                .margin({ right: 10 })
            }.width('100%')
          }.width('100%')
          .onClick(() => {
            MKNavPathStack.pushPathByName('ChangeInfoView', "用户名");
          })

          ListItem() {
            Row() {
              Text("性别").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
              Blank();
              Text(this.gender).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 2 })
                .fontColor($r('app.color.common_gray_01'))
              Image($r('app.media.ic_public_arrow_right'))
                .height(26)
                .fillColor($r('app.color.common_gray_01'))
                .margin({ right: 10 })
            }.width('100%')
          }.width('100%')
          .onClick(() => {
            MKNavPathStack.pushPathByName('ChangeInfoView', "性别");
          })

          ListItem() {
            Row() {
              Text("年龄").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
              Blank();
              Text(this.age.toString()).fontWeight(FontWeight.Medium).fontSize(16).margin({ right: 2 })
                .fontColor($r('app.color.common_gray_01'))
              Image($r('app.media.ic_public_arrow_right'))
                .height(26)
                .fillColor($r('app.color.common_gray_01'))
                .margin({ right: 10 })
            }.width('100%')
          }.width('100%')
          .onClick(() => {
            MKNavPathStack.pushPathByName('ChangeInfoView', "年龄");
          })

          ListItem() {
            Row() {
              Text("手机号").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
              Blank();
              Text(this.phone.replace(/(\d{3})\d{4}(\d{4})/, "$1****$2"))
                .fontWeight(FontWeight.Medium)
                .fontSize(16)
                .margin({ right: 2 })
                .fontColor($r('app.color.common_gray_01'))
              Image($r('app.media.ic_public_arrow_right'))
                .height(26)
                .fillColor($r('app.color.common_gray_01'))
                .margin({ right: 10 })
            }.width('100%')
          }.width('100%')
          .onClick(() => {
            MKNavPathStack.pushPathByName('ChangeInfoView', "手机号");
          })

          ListItem() {
            Row() {
              Text("生日").fontWeight(FontWeight.Medium).fontSize(16).margin({ left: 16 })
              Blank();
              Text(this.formatDate(this.birth_date))
                .fontWeight(FontWeight.Medium)
                .fontSize(16)
                .margin({ right: 2 })
                .fontColor($r('app.color.common_gray_01'))
              Image($r('app.media.ic_public_arrow_right'))
                .height(26)
                .fillColor($r('app.color.common_gray_01'))
                .margin({ right: 10 })
            }.width('100%')
          }.width('100%')
          .onClick(() => {
            MKNavPathStack.pushPathByName('ChangeInfoView', "生日");
          })
        }
        .height(270)
        .width('100%')
        .margin({ top: 26 })
        .backgroundColor($r('app.color.white'))
        .padding({ top: 18, bottom: 18 })
        .divider({
          strokeWidth: 1, //分割线宽度
          color: '#ffe9f0f0'   //分隔线颜色
        })

        Row({ space: 5 }) {
          Image('https://i.postimg.cc/bwnbgH1z/cancel.png').height(26).margin({ left: 16 })
          Text('注销账号').fontWeight(FontWeight.Medium).fontSize(16)
          Blank()
          Image($r('app.media.ic_public_arrow_right'))
            .height(26)
            .fillColor($r('app.color.common_gray_01'))
            .margin({ right: 10 })
        }
        .width('100%')
        .height(66)
        .backgroundColor($r('app.color.white'))
        .onClick(async () => {
          if (userPref.getValue('token') !== '') {
            this.getUIContext().showAlertDialog({
              title: `注销账号`,
              message: "是否确定注销账号",
              buttons: [
                {
                  value: '取消',
                  fontColor: $r('app.color.red'),
                  action: () => {
                    console.log('点击了取消');
                  }
                },
                {
                  value: '确定',
                  fontColor: $r('app.color.common_gray_01'),
                  action: async () => {
                    this.dialog.open()
                    try {
                      const res = await deleteRegisterApi(userPref.getValue('userid'))
                      if (res.data.code === 200) {
                        //删除首选项持久化数据
                        await userPref.deleteValue('token');
                        await userPref.deleteValue('userid');
                        await userPref.deleteValue('phone');
                        await userPref.deleteValue('username');
                        await userPref.deleteValue('avatar_url');
                        await userPref.deleteValue('token_expire');
                        //将头像和用户名恢复默认
                        AppStorage.setOrCreate('username', '请登录')
                        AppStorage.setOrCreate('avatar_url', 'https://i.postimg.cc/zfXht4P7/profile.png')
                        AppStorage.setOrCreate('isLoggedIn', false);
                        MqttClientWrapper.destroy() //销毁客户端
                        MKNavPathStack.pop()
                        promptAction.openToast({ message: '注销成功' })
                        this.dialog.close()
                      } else {
                        promptAction.openToast({ message: '注销失败' })
                        this.dialog.close()
                      }
                    } catch (error) {
                      // 主要处理500错误和网络错误
                      console.error('注销错误：', error);
                      promptAction.openToast({ message: '服务器错误，请稍后重试' })
                      this.dialog.close()
                    }
                  }
                }
              ]
            });
          }
        })
      }.width('100%').padding({ top: 16, bottom: 16 })
    }
    .height('100%')
    .width('100%')
    .scrollBar(BarState.Off)
    .backgroundColor($r('app.color.common_gray_bg'))
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
  }
}

export default UserInfoView