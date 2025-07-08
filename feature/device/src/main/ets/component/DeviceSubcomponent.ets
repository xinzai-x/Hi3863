import { promptAction } from "@kit.ArkUI"

//设备UI小组件
@Component
struct deviceSubcomponent {
  //接收父组件传进的参数
  @Prop title: string = '默认标题'
  // @Prop status: string = ''
  @Prop location: string = '客厅'
  @Prop linkMode: string = 'MQTT'
  @Prop icon: string = ''

  build() {
    Column({ space: 5 }) {
      Text(this.title)
        .maxLines(1)
        .textOverflow({ overflow: TextOverflow.Ellipsis })
        .fontSize(18).fontWeight(600)
      Row({ space: 1 }) {
        Text(`位置:${this.location}`)
          .fontWeight(FontWeight.Medium)
          .fontSize(12).fontColor($r('app.color.common_gray_02'))
      }

      Row() {
        Image(this.icon)
          .width('50%').padding(4)
        Row() {
          Image($r('app.media.ic_public_arrow_right'))
            .height(30).height(30)
        }
        .width(40)
        .height(40)
        .borderRadius(20)
        .padding(5)
        .alignItems(VerticalAlign.Center)
        .backgroundColor($r('app.color.home_gray'))
      }.layoutWeight(1).width('100%')
      .alignItems(VerticalAlign.Bottom)
      .justifyContent(FlexAlign.SpaceBetween)
    }
    .height(160)
    .width('100%')
    .padding(12)
    .borderRadius(20)
    .opacity(0.95)
    .backgroundColor($r('app.color.white'))
    .alignItems(HorizontalAlign.Start)
    .shadow({
      //添加阴影
      radius: 20,
      color: '#26000000', // 华为推荐工具栏阴影
      offsetY: 2
    })

  }
}

export default deviceSubcomponent