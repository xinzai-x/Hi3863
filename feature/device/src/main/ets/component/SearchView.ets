import {
  deviceBasicInfo,
  InfoParams,
  getIconByType,
  getSearchDeviceApi,
  MKNavPathStack,
  SimpleDataSource
} from 'basic'
import { promptAction } from '@kit.ArkUI'
import deviceSubcomponent from './DeviceSubcomponent';

@Preview
@Component
export struct SearchView {
  @StorageProp('safeTop') safeTop: number = 0
  @State keyword: string = ''
  // 使用统一封装的数据源
  @State dataSource: SimpleDataSource<deviceBasicInfo> = new SimpleDataSource<deviceBasicInfo>([]);

  build() {
    Column() {
      // search搜索框
      Row() {
        Image($r('app.media.ic_public_arrow_left'))
          .width(24)
          .aspectRatio(1)
          .fillColor(Color.White)
          .margin(13)
          .onClick(() => {
            MKNavPathStack.pop()
          })
        Search({ placeholder: '设备名字...', value: $$this.keyword })
          .searchIcon({ src: $r('app.media.ic_public_search'), color: $r('app.color.common_gray_01') })
          .placeholderColor($r('app.color.common_gray_02'))
          .placeholderFont({ size: 14 })
          .searchButton('搜索', { fontSize: 14, fontColor: $r('app.color.black') })
          .backgroundColor($r('app.color.white'))
          .textFont({ size: 14 })
          .layoutWeight(1)
          .height(36)
          .caretStyle({ color: $r('app.color.red') })
          .defaultFocus(true)
          .onSubmit(async () => {
            try {
              const res = await getSearchDeviceApi(this.keyword)
              this.dataSource.updateData(res.data.data) //更新设备数据
              console.log(`搜索结果：${JSON.stringify(res)}`)
            } catch (error) {
              console.error('搜索错误：', error);
              if (error.response) {
                switch (error.response.status) {
                  case 400:
                    promptAction.openToast({ message: '请求参数无效，请检查输入信息' })
                    break;
                  case 401:
                    promptAction.openToast({ message: '无有效token' })
                    break;
                  case 404:
                    promptAction.openToast({ message: '未找到匹配的设备' })
                    break;
                  case 500:
                    promptAction.openToast({ message: '服务器内部错误，请稍后重试' })
                    break;
                  default:
                    promptAction.openToast({ message: '搜索失败，请稍后重试' })
                    break;
                }
              }
            }
          })
      }
      .padding({ top: this.safeTop, right: 16 })
      .linearGradient({
        angle: 135,
        colors: [[$r('app.color.red'), 0], [$r('app.color.common_gray_01'), 1]]
      })
      .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.TOP])

      Column() {
        Scroll() {
          Column({ space: 5 }) {
            GridRow({
              columns: {
                xs: 1,
                sm: 2,
                md: 4,
                lg: 6,
                xl: 8,
                xxl: 10
              },
              gutter: 12
            }) {
              LazyForEach(this.dataSource, (item: deviceBasicInfo) => {
                GridCol() {
                  deviceSubcomponent({
                    title: item.name,
                    location: item.location,
                    linkMode: item.protocol,
                    icon: getIconByType(item.type),
                  })
                }
                .onClick(async () => {
                  if (item.type === '温湿度传感器') {
                    if (item.protocol === 'MQTT') {
                      //传入子页面的参数
                      const params: InfoParams = {
                        deviceId: item.id,
                        deviceName: item.name,
                        icon: getIconByType(item.type),
                        linkMode: item.protocol,
                      };
                      console.log(JSON.stringify(params))
                      MKNavPathStack.pushPathByName('DHTInfoView', params)
                    } else if (item.protocol === 'HTTP') {
                      promptAction.openToast({ message: '暂不支持' })
                    }
                  } else if (item.type === 'LED灯') {
                    if (item.protocol === 'MQTT') {
                      //传入子页面的参数
                      const params: InfoParams = {
                        deviceId: item.id,
                        deviceName: item.name,
                        icon: getIconByType(item.type),
                        linkMode: item.protocol,
                      };
                      console.log(JSON.stringify(params))
                      MKNavPathStack.pushPathByName('LEDInfoView', params)
                    } else if (item.protocol === 'HTTP') {
                      promptAction.openToast({ message: '暂不支持' })
                    }
                  }
                })
              }, (item: deviceBasicInfo) => item.id)
            }
          }.width('100%').align(Alignment.TopStart)
        }.width('100%').padding(16).margin({ bottom: 30 })
      }.width('100%').height('100%')
    }
    .width('100%')
    .height('100%')
    .backgroundColor($r('app.color.common_gray_bg'))
    .expandSafeArea([SafeAreaType.SYSTEM], [SafeAreaEdge.BOTTOM]) //隐藏状态和控制栏
  }
}