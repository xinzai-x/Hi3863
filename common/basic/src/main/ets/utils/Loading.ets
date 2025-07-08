// 加载小组件封装
@CustomDialog
export struct Loading {
  message: string = '加载中'
  controller: CustomDialogController

  build() {
    Column({ space: 10 }) {
      LoadingProgress()
        .width(48)
        .height(48)
        .color(Color.White)
      if (this.message) {
        Text(this.message)
          .fontSize(14)
          .fontColor(Color.White)
      }
    }
    .justifyContent(FlexAlign.Center)
    .width(120)
    .height(120)
    .backgroundColor('rgba(0,0,0,0.6)')
    .borderRadius(16)
  }
}
