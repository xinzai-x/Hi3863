import { MKNavPathStack } from "basic";

// 设置页面
@Entry
@Component
struct InstallView {
  @State message: string = '设置页面';

  build() {
    Column() {
      Row() {
        Text('修改密码').fontWeight(FontWeight.Medium).margin({ left: 16 })
        Blank();
        Image($r('app.media.ic_public_arrow_right'))
          .height(26)
          .fillColor($r('app.color.common_gray_01'))
          .margin({ right: 10 })
      }.width('100%').height(60).backgroundColor($r('app.color.white'))
      .onClick(() => {
        MKNavPathStack.pushPathByName('ChangePassWordView', null)
      })
    }
    .width('100%')
    .height('100%')
    .padding({ top: 16, bottom: 16 })
    .backgroundColor($r('app.color.common_gray_bg'))
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
  }
}

export default InstallView